/******************************************************************************
 * Preemptive Global Earliest Deadline First  (EDF) scheduler for Xen
 * EDF scheduling is a real-time scheduling algorithm used in embedded field.
 *
 * by Sisu Xi, 2013, Washington University in Saint Louis
 * and Meng Xu, 2014, University of Pennsylvania
 *
 * based on the code of credit Scheduler
 */

#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/perfc.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <xen/errno.h>
#include <xen/trace.h>
#include <xen/cpu.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>
#include <xen/guest_access.h>

/*
 * TODO:
 *
 * Migration compensation and resist like credit2 to better use cache;
 * Lock Holder Problem, using yield?
 * Self switch problem: VCPUs of the same domain may preempt each other;
 */

/*
 * Design:
 *
 * This scheduler follows the Preemptive Global Earliest Deadline First (EDF)
 * theory in real-time field.
 * At any scheduling point, the VCPU with earlier deadline has higher priority.
 * The scheduler always picks highest priority VCPU to run on a feasible PCPU.
 * A PCPU is feasible if the VCPU can run on this PCPU and (the PCPU is idle or
 * has a lower-priority VCPU running on it.)
 * 
 * Each VCPU has a dedicated period and budget.
 * The deadline of a VCPU is at the end of each of its periods;
 * A VCPU has its budget replenished at the beginning of each of its periods;
 * While scheduled, a VCPU burns its budget.
 * The VCPU needs to finish its budget before its deadline in each period;
 * The VCPU discards its unused budget at the end of each of its periods.
 * If a VCPU runs out of budget in a period, it has to wait until next period.
 * 
 * Each VCPU is implemented as a deferable server.
 * When a VCPU has a task running on it, its budget is continuously burned;
 * When a VCPU has no task but with budget left, its budget is preserved.
 *
 * Queue scheme: A global runqueue for each CPU pool. 
 * The runqueue holds all runnable VCPUs. 
 * VCPUs in the runqueue are divided into two parts: 
 * with and without remaining budget. 
 * At the first part, VCPUs are sorted based on EDF priority scheme.
 *
 * Note: cpumask and cpupool is supported.
 */

/*
 * Locking:
 * A global system lock is used to protect the RunQ.
 * The global lock is referenced by schedule_data.schedule_lock 
 * from all physical cpus.
 *
 * The lock is already grabbed when calling wake/sleep/schedule/ functions 
 * in schedule.c
 *
 * The functions involes RunQ and needs to grab locks are:
 *    vcpu_insert, vcpu_remove, context_saved, __runq_insert
 */


/*
 * Default parameters: 
 * Period and budget in default is 10 and 4 ms, respectively
 */
#define RT_DS_DEFAULT_PERIOD     (MICROSECS(10000))
#define RT_DS_DEFAULT_BUDGET     (MICROSECS(4000))

/*
 * Flags
 */ 
/*
 * RT_scheduled: Is this vcpu either running on, or context-switching off,
 * a phyiscal cpu?
 * + Accessed only with Runqueue lock held.
 * + Set when chosen as next in rt_schedule().
 * + Cleared after context switch has been saved in rt_context_saved()
 * + Checked in vcpu_wake to see if we can add to the Runqueue, or if we should
 *   set RT_delayed_runq_add
 * + Checked to be false in runq_insert.
 */
#define __RT_scheduled            1
#define RT_scheduled (1<<__RT_scheduled)
/* 
 * RT_delayed_runq_add: Do we need to add this to the Runqueueu once it'd done 
 * being context switching out?
 * + Set when scheduling out in rt_schedule() if prev is runable
 * + Set in rt_vcpu_wake if it finds RT_scheduled set
 * + Read in rt_context_saved(). If set, it adds prev to the Runqueue and
 *   clears the bit.
 */
#define __RT_delayed_runq_add     2
#define RT_delayed_runq_add (1<<__RT_delayed_runq_add)

/*
 * Debug only. Used to printout debug information
 */
#define printtime()\
        ({s_time_t now = NOW(); \
          printk("%u : %3ld.%3ldus : %-19s\n",smp_processor_id(),\
          now/MICROSECS(1), now%MICROSECS(1)/1000, __func__);} )

/*
 * rt tracing events ("only" 512 available!). Check
 * include/public/trace.h for more details.
 */
#define TRC_RT_TICKLE           TRC_SCHED_CLASS_EVT(RT, 1)
#define TRC_RT_RUNQ_PICK        TRC_SCHED_CLASS_EVT(RT, 2)
#define TRC_RT_BUDGET_BURN      TRC_SCHED_CLASS_EVT(RT, 3)
#define TRC_RT_BUDGET_REPLENISH TRC_SCHED_CLASS_EVT(RT, 4)
#define TRC_RT_SCHED_TASKLET    TRC_SCHED_CLASS_EVT(RT, 5)
#define TRC_RT_VCPU_DUMP        TRC_SCHED_CLASS_EVT(RT, 6)

/*
 * Systme-wide private data, include a global RunQueue
 * Global lock is referenced by schedule_data.schedule_lock from all 
 * physical cpus. It can be grabbed via vcpu_schedule_lock_irq()
 */
struct rt_private {
    spinlock_t lock;           /* The global coarse grand lock */
    struct list_head sdom;     /* list of availalbe domains, used for dump */
    struct list_head runq;     /* Ordered list of runnable VMs */
    struct rt_vcpu *flag_vcpu; /* position of the first depleted vcpu */
    cpumask_t cpus;            /* cpumask_t of available physical cpus */
    cpumask_t tickled;         /* cpus been tickled */
};

/*
 * Virtual CPU
 */
struct rt_vcpu {
    struct list_head runq_elem; /* On the runqueue list */
    struct list_head sdom_elem; /* On the domain VCPU list */

    /* Up-pointers */
    struct rt_dom *sdom;
    struct vcpu *vcpu;

    /* VCPU parameters, in nanoseconds */
    s_time_t period;
    s_time_t budget;

    /* VCPU current infomation in nanosecond */
    s_time_t cur_budget;        /* current budget */
    s_time_t last_start;        /* last start time */
    s_time_t cur_deadline;      /* current deadline for EDF */

    unsigned flags;             /* mark __RT_scheduled, etc.. */
};

/*
 * Domain
 */
struct rt_dom {
    struct list_head vcpu;      /* link its VCPUs */
    struct list_head sdom_elem; /* link list on rt_priv */
    struct domain *dom;         /* pointer to upper domain */
};

/*
 * Useful inline functions
 */
static inline struct rt_private *RT_PRIV(const struct scheduler *ops)
{
    return ops->sched_data;
}

static inline struct rt_vcpu *RT_VCPU(const struct vcpu *vcpu)
{
    return vcpu->sched_priv;
}

static inline struct rt_dom *RT_DOM(const struct domain *dom)
{
    return dom->sched_priv;
}

static inline struct list_head *RUNQ(const struct scheduler *ops)
{
    return &RT_PRIV(ops)->runq;
}

/*
 * RunQueue helper functions
 */
static int
__vcpu_on_runq(const struct rt_vcpu *svc)
{
   return !list_empty(&svc->runq_elem);
}

static struct rt_vcpu *
__runq_elem(struct list_head *elem)
{
    return list_entry(elem, struct rt_vcpu, runq_elem);
}

/*
 * Debug related code, dump vcpu/cpu information
 */
static void
rt_dump_vcpu(const struct scheduler *ops, const struct rt_vcpu *svc)
{
    struct rt_private *prv = RT_PRIV(ops);
    char cpustr[1024];
    cpumask_t *cpupool_mask;

    ASSERT(svc != NULL);
    /* flag vcpu */
    if( svc->sdom == NULL )
        return;

    cpumask_scnprintf(cpustr, sizeof(cpustr), svc->vcpu->cpu_hard_affinity);
    printk("[%5d.%-2u] cpu %u, (%"PRI_stime", %"PRI_stime"),"
           " cur_b=%"PRI_stime" cur_d=%"PRI_stime" last_start=%"PRI_stime
           " onR=%d runnable=%d cpu_hard_affinity=%s ",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->vcpu->processor,
            svc->period,
            svc->budget,
            svc->cur_budget,
            svc->cur_deadline,
            svc->last_start,
            __vcpu_on_runq(svc),
            vcpu_runnable(svc->vcpu),
            cpustr);
    memset(cpustr, 0, sizeof(cpustr));
    cpupool_mask = cpupool_scheduler_cpumask(svc->vcpu->domain->cpupool);
    cpumask_scnprintf(cpustr, sizeof(cpustr), cpupool_mask);
    printk("cpupool=%s ", cpustr);
    memset(cpustr, 0, sizeof(cpustr));
    cpumask_scnprintf(cpustr, sizeof(cpustr), &prv->cpus);
    printk("prv->cpus=%s\n", cpustr);
    
    /* TRACE */
    {
        struct {
            unsigned dom:16,vcpu:16;
            unsigned processor;
            unsigned cur_budget_lo, cur_budget_hi;
            unsigned cur_deadline_lo, cur_deadline_hi;
            unsigned is_vcpu_on_runq:16,is_vcpu_runnable:16;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.processor = svc->vcpu->processor;
        d.cur_budget_lo = (unsigned) svc->cur_budget;
        d.cur_budget_hi = (unsigned) (svc->cur_budget >> 32);
        d.cur_deadline_lo = (unsigned) svc->cur_deadline;
        d.cur_deadline_hi = (unsigned) (svc->cur_deadline >> 32);
        d.is_vcpu_on_runq = __vcpu_on_runq(svc);
        d.is_vcpu_runnable = vcpu_runnable(svc->vcpu);
        trace_var(TRC_RT_VCPU_DUMP, 1,
                  sizeof(d),
                  (unsigned char *)&d);
    }
}

static void
rt_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct rt_vcpu *svc = RT_VCPU(curr_on_cpu(cpu));

    printtime();
    rt_dump_vcpu(ops, svc);
}

/*
 * should not need lock here. only showing stuff 
 */
static void
rt_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom, *iter_svc, *runq, *iter;
    struct rt_private *prv = RT_PRIV(ops);
    struct rt_vcpu *svc;
    unsigned int cpu = 0;

    printtime();

    printk("PCPU info:\n");
    for_each_cpu(cpu, &prv->cpus) 
        rt_dump_pcpu(ops, cpu);

    printk("Global RunQueue info:\n");
    runq = RUNQ(ops);
    list_for_each( iter, runq ) 
    {
        svc = __runq_elem(iter);
        rt_dump_vcpu(ops, svc);
    }

    printk("Domain info:\n");
    list_for_each( iter_sdom, &prv->sdom ) 
    {
        struct rt_dom *sdom;
        sdom = list_entry(iter_sdom, struct rt_dom, sdom_elem);
        printk("\tdomain: %d\n", sdom->dom->domain_id);

        list_for_each( iter_svc, &sdom->vcpu ) 
        {
            svc = list_entry(iter_svc, struct rt_vcpu, sdom_elem);
            rt_dump_vcpu(ops, svc);
        }
    }

    printk("\n");
}

/*
 * update deadline and budget when deadline is in the past,
 * it need to be updated to the deadline of the current period 
 */
static void
rt_update_helper(s_time_t now, struct rt_vcpu *svc)
{
    s_time_t diff = now - svc->cur_deadline;

    if ( diff >= 0 ) 
    {
        /* now can be later for several periods */
        long count = ( diff/svc->period ) + 1;
        svc->cur_deadline += count * svc->period;
        svc->cur_budget = svc->budget;

        /* TRACE */
        {
            struct {
                unsigned dom:16,vcpu:16;
                unsigned cur_budget_lo, cur_budget_hi;
            } d;
            d.dom = svc->vcpu->domain->domain_id;
            d.vcpu = svc->vcpu->vcpu_id;
            d.cur_budget_lo = (unsigned) svc->cur_budget;
            d.cur_budget_hi = (unsigned) (svc->cur_budget >> 32);
            trace_var(TRC_RT_BUDGET_REPLENISH, 1,
                      sizeof(d),
                      (unsigned char *) &d);
        }

        return;
    }
}

static inline void
__runq_remove(struct rt_vcpu *svc)
{
    if ( __vcpu_on_runq(svc) )
        list_del_init(&svc->runq_elem);
}

/*
 * Insert svc in the RunQ according to EDF: vcpus with smaller deadlines
 * goes first.
 */
static void
__runq_insert(const struct scheduler *ops, struct rt_vcpu *svc)
{
    struct rt_private *prv = RT_PRIV(ops);
    struct list_head *runq = RUNQ(ops);
    struct list_head *iter;
    spinlock_t *schedule_lock;
    
    schedule_lock = per_cpu(schedule_data, svc->vcpu->processor).schedule_lock;
    ASSERT( spin_is_locked(schedule_lock) );
    
    ASSERT( !__vcpu_on_runq(svc) );

    /* svc still has budget */
    if ( svc->cur_budget > 0 ) 
    {
        list_for_each(iter, runq) 
        {
            struct rt_vcpu * iter_svc = __runq_elem(iter);
            if ( iter_svc->cur_budget == 0 ||
                 svc->cur_deadline <= iter_svc->cur_deadline )
                    break;
         }
        list_add_tail(&svc->runq_elem, iter);
     }
    else 
    {
        list_add(&svc->runq_elem, &prv->flag_vcpu->runq_elem);
    }
}

/*
 * Init/Free related code
 */
static int
rt_init(struct scheduler *ops)
{
    struct rt_private *prv = xzalloc(struct rt_private);

    printk("Initializing RT scheduler\n" \
           " WARNING: This is experimental software in development.\n" \
           " Use at your own risk.\n");

    if ( prv == NULL )
        return -ENOMEM;

    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->sdom);
    INIT_LIST_HEAD(&prv->runq);

    prv->flag_vcpu = xzalloc(struct rt_vcpu);
    prv->flag_vcpu->cur_budget = 0;
    prv->flag_vcpu->sdom = NULL; /* distinguish this vcpu with others */
    list_add(&prv->flag_vcpu->runq_elem, &prv->runq);

    cpumask_clear(&prv->cpus);
    cpumask_clear(&prv->tickled);

    ops->sched_data = prv;

    printtime();
    printk("\n");

    return 0;
}

static void
rt_deinit(const struct scheduler *ops)
{
    struct rt_private *prv = RT_PRIV(ops);

    printtime();
    printk("\n");
    xfree(prv->flag_vcpu);
    xfree(prv);
}

/* 
 * Point per_cpu spinlock to the global system lock;
 * All cpu have same global system lock 
 */
static void *
rt_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct rt_private *prv = RT_PRIV(ops);

    cpumask_set_cpu(cpu, &prv->cpus);

    per_cpu(schedule_data, cpu).schedule_lock = &prv->lock;

    printtime();
    printk("%s total cpus: %d", __func__, cpumask_weight(&prv->cpus));
    /* 1 indicates alloc. succeed in schedule.c */
    return (void *)1;
}

static void
rt_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct rt_private * prv = RT_PRIV(ops);
    cpumask_clear_cpu(cpu, &prv->cpus);
}

static void *
rt_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
    unsigned long flags;
    struct rt_dom *sdom;
    struct rt_private * prv = RT_PRIV(ops);

    sdom = xzalloc(struct rt_dom);
    if ( sdom == NULL ) 
    {
        printk("%s, xzalloc failed\n", __func__);
        return NULL;
    }

    INIT_LIST_HEAD(&sdom->vcpu);
    INIT_LIST_HEAD(&sdom->sdom_elem);
    sdom->dom = dom;

    /* spinlock here to insert the dom */
    spin_lock_irqsave(&prv->lock, flags);
    list_add_tail(&sdom->sdom_elem, &(prv->sdom));
    spin_unlock_irqrestore(&prv->lock, flags);

    return sdom;
}

static void
rt_free_domdata(const struct scheduler *ops, void *data)
{
    unsigned long flags;
    struct rt_dom *sdom = data;
    struct rt_private *prv = RT_PRIV(ops);

    spin_lock_irqsave(&prv->lock, flags);
    list_del_init(&sdom->sdom_elem);
    spin_unlock_irqrestore(&prv->lock, flags);
    xfree(data);
}

static int
rt_dom_init(const struct scheduler *ops, struct domain *dom)
{
    struct rt_dom *sdom;

    /* IDLE Domain does not link on rt_private */
    if ( is_idle_domain(dom) ) 
        return 0;

    sdom = rt_alloc_domdata(ops, dom);
    if ( sdom == NULL ) 
    {
        printk("%s, failed\n", __func__);
        return -ENOMEM;
    }
    dom->sched_priv = sdom;

    return 0;
}

static void
rt_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
    rt_free_domdata(ops, RT_DOM(dom));
}

static void *
rt_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct rt_vcpu *svc;
    s_time_t now = NOW();

    /* Allocate per-VCPU info */
    svc = xzalloc(struct rt_vcpu);
    if ( svc == NULL ) 
    {
        printk("%s, xzalloc failed\n", __func__);
        return NULL;
    }

    INIT_LIST_HEAD(&svc->runq_elem);
    INIT_LIST_HEAD(&svc->sdom_elem);
    svc->flags = 0U;
    svc->sdom = dd;
    svc->vcpu = vc;
    svc->last_start = 0;

    svc->period = RT_DS_DEFAULT_PERIOD;
    if ( !is_idle_vcpu(vc) )
        svc->budget = RT_DS_DEFAULT_BUDGET;

    rt_update_helper(now, svc);

    /* Debug only: dump new vcpu's info */
    rt_dump_vcpu(ops, svc);

    return svc;
}

static void
rt_free_vdata(const struct scheduler *ops, void *priv)
{
    struct rt_vcpu *svc = priv;

    /* Debug only: dump freed vcpu's info */
    rt_dump_vcpu(ops, svc);
    xfree(svc);
}

/*
 * This function is called in sched_move_domain() in schedule.c
 * When move a domain to a new cpupool.
 * It inserts vcpus of moving domain to the scheduler's RunQ in
 * dest. cpupool; and insert rt_vcpu svc to scheduler-specific
 * vcpu list of the dom
 */
static void
rt_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct rt_vcpu *svc = RT_VCPU(vc);

    /* Debug only: dump info of vcpu to insert */
    rt_dump_vcpu(ops, svc);

    /* not addlocate idle vcpu to dom vcpu list */
    if ( is_idle_vcpu(vc) )
        return;

    if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running )
        __runq_insert(ops, svc);

    /* add rt_vcpu svc to scheduler-specific vcpu list of the dom */
    list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);
}

/*
 * Remove rt_vcpu svc from the old scheduler in source cpupool; and
 * Remove rt_vcpu svc from scheduler-specific vcpu list of the dom
 */
static void
rt_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
    struct rt_vcpu * const svc = RT_VCPU(vc);
    struct rt_dom * const sdom = svc->sdom;

    rt_dump_vcpu(ops, svc);

    BUG_ON( sdom == NULL );
    BUG_ON( __vcpu_on_runq(svc) );

    if ( __vcpu_on_runq(svc) )
        __runq_remove(svc);

    if ( !is_idle_vcpu(vc) ) 
        list_del_init(&svc->sdom_elem);
}

/* 
 * Pick a valid CPU for the vcpu vc
 * Valid CPU of a vcpu is intesection of vcpu's affinity 
 * and available cpus
 */
static int
rt_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    cpumask_t cpus;
    cpumask_t *online;
    int cpu;
    struct rt_private * prv = RT_PRIV(ops);

    online = cpupool_scheduler_cpumask(vc->domain->cpupool);
    cpumask_and(&cpus, &prv->cpus, online);
    cpumask_and(&cpus, &cpus, vc->cpu_hard_affinity);

    cpu = cpumask_test_cpu(vc->processor, &cpus)
            ? vc->processor 
            : cpumask_cycle(vc->processor, &cpus);
    ASSERT( !cpumask_empty(&cpus) && cpumask_test_cpu(cpu, &cpus) );

    return cpu;
}

/*
 * Burn budget in nanosecond granularity
 */
static void
burn_budgets(const struct scheduler *ops, struct rt_vcpu *svc, s_time_t now) 
{
    s_time_t delta;

    /* don't burn budget for idle VCPU */
    if ( is_idle_vcpu(svc->vcpu) ) 
        return;

    rt_update_helper(now, svc);

    /* not burn budget when vcpu miss deadline */
    if ( now >= svc->cur_deadline )
        return;

    /* burn at nanoseconds level */
    delta = now - svc->last_start;
    /* 
     * delta < 0 only happens in nested virtualization;
     * TODO: how should we handle delta < 0 in a better way? 
     */
    if ( delta < 0 ) 
    {
        printk("%s, ATTENTION: now is behind last_start! delta = %ld",
                __func__, delta);
        rt_dump_vcpu(ops, svc);
        svc->last_start = now;
        svc->cur_budget = 0;
        return;
    }

    if ( svc->cur_budget == 0 ) 
        return;

    svc->cur_budget -= delta;
    if ( svc->cur_budget < 0 ) 
        svc->cur_budget = 0;

    /* TRACE */
    {
        struct {
            unsigned dom:16, vcpu:16;
            unsigned cur_budget_lo;
            unsigned cur_budget_hi;
            int delta;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.cur_budget_lo = (unsigned) svc->cur_budget;
        d.cur_budget_hi = (unsigned) (svc->cur_budget >> 32);
        d.delta = delta;
        trace_var(TRC_RT_BUDGET_BURN, 1,
                  sizeof(d),
                  (unsigned char *) &d);
    }
}

/* 
 * RunQ is sorted. Pick first one within cpumask. If no one, return NULL
 * lock is grabbed before calling this function 
 */
static struct rt_vcpu *
__runq_pick(const struct scheduler *ops, cpumask_t mask)
{
    struct list_head *runq = RUNQ(ops);
    struct list_head *iter;
    struct rt_vcpu *svc = NULL;
    struct rt_vcpu *iter_svc = NULL;
    cpumask_t cpu_common;
    cpumask_t *online;
    struct rt_private * prv = RT_PRIV(ops);

    list_for_each(iter, runq) 
    {
        iter_svc = __runq_elem(iter);

        /* flag vcpu */
        if(iter_svc->sdom == NULL)
            break;

        /* mask cpu_hard_affinity & cpupool & priv->cpus */
        online = cpupool_scheduler_cpumask(iter_svc->vcpu->domain->cpupool);
        cpumask_and(&cpu_common, online, &prv->cpus);
        cpumask_and(&cpu_common, &cpu_common, iter_svc->vcpu->cpu_hard_affinity);
        cpumask_and(&cpu_common, &mask, &cpu_common);
        if ( cpumask_empty(&cpu_common) )
            continue;

        ASSERT( iter_svc->cur_budget > 0 );

        svc = iter_svc;
        break;
    }

    /* TRACE */
    {
        if( svc != NULL )
        {
            struct {
                unsigned dom:16, vcpu:16;
                unsigned cur_deadline_lo, cur_deadline_hi;
                unsigned cur_budget_lo, cur_budget_hi;
            } d;
            d.dom = svc->vcpu->domain->domain_id;
            d.vcpu = svc->vcpu->vcpu_id;
            d.cur_deadline_lo = (unsigned) svc->cur_deadline;
            d.cur_deadline_hi = (unsigned) (svc->cur_deadline >> 32);
            d.cur_budget_lo = (unsigned) svc->cur_budget;
            d.cur_budget_hi = (unsigned) (svc->cur_budget >> 32);
            trace_var(TRC_RT_RUNQ_PICK, 1,
                      sizeof(d),
                      (unsigned char *) &d);
        }
        else
            trace_var(TRC_RT_RUNQ_PICK, 1, 0, NULL);
    }

    return svc;
}

/*
 * Update vcpu's budget and sort runq by insert the modifed vcpu back to runq
 * lock is grabbed before calling this function 
 */
static void
__repl_update(const struct scheduler *ops, s_time_t now)
{
    struct list_head *runq = RUNQ(ops);
    struct list_head *iter;
    struct list_head *tmp;
    struct rt_vcpu *svc = NULL;

    list_for_each_safe(iter, tmp, runq) 
    {
        svc = __runq_elem(iter);

        /* not update flag_vcpu's budget */
        if(svc->sdom == NULL)
            continue;

        rt_update_helper(now, svc);
        /* reinsert the vcpu if its deadline is updated */
        if ( now >= 0 )
        {
            __runq_remove(svc);
            __runq_insert(ops, svc);
        }
    }
}

/* 
 * schedule function for rt scheduler.
 * The lock is already grabbed in schedule.c, no need to lock here 
 */
static struct task_slice
rt_schedule(const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    const int cpu = smp_processor_id();
    struct rt_private * prv = RT_PRIV(ops);
    struct rt_vcpu * const scurr = RT_VCPU(current);
    struct rt_vcpu * snext = NULL;
    struct task_slice ret = { .migrated = 0 };

    /* clear ticked bit now that we've been scheduled */
    if ( cpumask_test_cpu(cpu, &prv->tickled) )
        cpumask_clear_cpu(cpu, &prv->tickled);

    /* burn_budget would return for IDLE VCPU */
    burn_budgets(ops, scurr, now);

    __repl_update(ops, now);

    if ( tasklet_work_scheduled ) 
    {
        snext = RT_VCPU(idle_vcpu[cpu]);
    } 
    else 
    {
        cpumask_t cur_cpu;
        cpumask_clear(&cur_cpu);
        cpumask_set_cpu(cpu, &cur_cpu);
        snext = __runq_pick(ops, cur_cpu);
        if ( snext == NULL )
            snext = RT_VCPU(idle_vcpu[cpu]);

        /* if scurr has higher priority and budget, still pick scurr */
        if ( !is_idle_vcpu(current) &&
             vcpu_runnable(current) &&
             scurr->cur_budget > 0 &&
             ( is_idle_vcpu(snext->vcpu) ||
               scurr->cur_deadline <= snext->cur_deadline ) ) 
            snext = scurr;
    }

    if ( snext != scurr &&
         !is_idle_vcpu(current) &&
         vcpu_runnable(current) )
        set_bit(__RT_delayed_runq_add, &scurr->flags);
    

    snext->last_start = now;
    if ( !is_idle_vcpu(snext->vcpu) ) 
    {
        if ( snext != scurr ) 
        {
            __runq_remove(snext);
            set_bit(__RT_scheduled, &snext->flags);
        }
        if ( snext->vcpu->processor != cpu ) 
        {
            snext->vcpu->processor = cpu;
            ret.migrated = 1;
        }
    }

    ret.time = MILLISECS(1); /* sched quantum */
    ret.task = snext->vcpu;

    /* TRACE */
    {
        struct {
            unsigned dom:16,vcpu:16;
            unsigned cur_deadline_lo, cur_deadline_hi;
            unsigned cur_budget_lo, cur_budget_hi;
        } d;
        d.dom = snext->vcpu->domain->domain_id;
        d.vcpu = snext->vcpu->vcpu_id;
        d.cur_deadline_lo = (unsigned) snext->cur_deadline;
        d.cur_deadline_hi = (unsigned) (snext->cur_deadline >> 32);
        d.cur_budget_lo = (unsigned) snext->cur_budget;
        d.cur_budget_hi = (unsigned) (snext->cur_budget >> 32);
        trace_var(TRC_RT_SCHED_TASKLET, 1,
                  sizeof(d),
                  (unsigned char *)&d);
    }

    return ret;
}

/*
 * Remove VCPU from RunQ
 * The lock is already grabbed in schedule.c, no need to lock here 
 */
static void
rt_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
    struct rt_vcpu * const svc = RT_VCPU(vc);

    BUG_ON( is_idle_vcpu(vc) );

    if ( curr_on_cpu(vc->processor) == vc ) 
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);
    else if ( __vcpu_on_runq(svc) ) 
        __runq_remove(svc);
    else if ( test_bit(__RT_delayed_runq_add, &svc->flags) )
        clear_bit(__RT_delayed_runq_add, &svc->flags);
}

/*
 * Pick a vcpu on a cpu to kick out to place the running candidate
 * Called by wake() and context_saved()
 * We have a running candidate here, the kick logic is:
 * Among all the cpus that are within the cpu affinity
 * 1) if the new->cpu is idle, kick it. This could benefit cache hit
 * 2) if there are any idle vcpu, kick it.
 * 3) now all pcpus are busy, among all the running vcpus, pick lowest priority one
 *    if snext has higher priority, kick it.
 *
 * TODO:
 * 1) what if these two vcpus belongs to the same domain?
 *    replace a vcpu belonging to the same domain introduces more overhead
 *
 * lock is grabbed before calling this function 
 */
static void
runq_tickle(const struct scheduler *ops, struct rt_vcpu *new)
{
    struct rt_private * prv = RT_PRIV(ops);
    struct rt_vcpu * latest_deadline_vcpu = NULL;    /* lowest priority scheduled */
    struct rt_vcpu * iter_svc;
    struct vcpu * iter_vc;
    int cpu = 0, cpu_to_tickle = 0;
    cpumask_t not_tickled;
    cpumask_t *online;

    if ( new == NULL || is_idle_vcpu(new->vcpu) ) 
        return;

    online = cpupool_scheduler_cpumask(new->vcpu->domain->cpupool);
    cpumask_and(&not_tickled, online, &prv->cpus);
    cpumask_and(&not_tickled, &not_tickled, new->vcpu->cpu_hard_affinity);
    cpumask_andnot(&not_tickled, &not_tickled, &prv->tickled);

    /* 1) if new's previous cpu is idle, kick it for cache benefit */
    if ( is_idle_vcpu(curr_on_cpu(new->vcpu->processor)) ) 
    {
        cpu_to_tickle = new->vcpu->processor;
        goto out;
    }

    /* 2) if there are any idle pcpu, kick it */
    /* The same loop also find the one with lowest priority */
    for_each_cpu(cpu, &not_tickled) 
    {
        iter_vc = curr_on_cpu(cpu);
        if ( is_idle_vcpu(iter_vc) ) 
        {
            cpu_to_tickle = cpu;
            goto out;
        }
        iter_svc = RT_VCPU(iter_vc);
        if ( latest_deadline_vcpu == NULL || 
             iter_svc->cur_deadline > latest_deadline_vcpu->cur_deadline )
            latest_deadline_vcpu = iter_svc;
    }

    /* 3) candicate has higher priority, kick out the lowest priority vcpu */
    if ( latest_deadline_vcpu != NULL && new->cur_deadline < latest_deadline_vcpu->cur_deadline ) 
    {
        cpu_to_tickle = latest_deadline_vcpu->vcpu->processor;
        goto out;
    }

out:
    /* TRACE */ 
    {
        struct {
            unsigned cpu:8, pad:24;
        } d;
        d.cpu = cpu_to_tickle;
        d.pad = 0;
        trace_var(TRC_RT_TICKLE, 0,
                  sizeof(d),
                  (unsigned char *)&d);
    }

    cpumask_set_cpu(cpu_to_tickle, &prv->tickled);
    cpu_raise_softirq(cpu_to_tickle, SCHEDULE_SOFTIRQ);
    return;    
}

/* 
 * Should always wake up runnable vcpu, put it back to RunQ. 
 * Check priority to raise interrupt 
 * The lock is already grabbed in schedule.c, no need to lock here 
 * TODO: what if these two vcpus belongs to the same domain? 
 */
static void
rt_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
    struct rt_vcpu * const svc = RT_VCPU(vc);
    s_time_t now = NOW();
    struct rt_private * prv = RT_PRIV(ops);
    struct rt_vcpu * snext = NULL;        /* highest priority on RunQ */

    BUG_ON( is_idle_vcpu(vc) );

    if ( unlikely(curr_on_cpu(vc->processor) == vc) ) 
        return;

    /* on RunQ, just update info is ok */
    if ( unlikely(__vcpu_on_runq(svc)) ) 
        return;

    /* If context hasn't been saved for this vcpu yet, we can't put it on
     * the Runqueue. Instead, we set a flag so that it will be put on the Runqueue
     * After the context has been saved. 
     */
    if ( unlikely(test_bit(__RT_scheduled, &svc->flags)) ) 
    {
        set_bit(__RT_delayed_runq_add, &svc->flags);
        return;
    }

    rt_update_helper(now, svc);

    __runq_insert(ops, svc);
    __repl_update(ops, now);
    snext = __runq_pick(ops, prv->cpus);    /* pick snext from ALL valid cpus */
    runq_tickle(ops, snext);

    return;
}

/* 
 * scurr has finished context switch, insert it back to the RunQ,
 * and then pick the highest priority vcpu from runq to run 
 */
static void
rt_context_saved(const struct scheduler *ops, struct vcpu *vc)
{
    struct rt_vcpu * svc = RT_VCPU(vc);
    struct rt_vcpu * snext = NULL;
    struct rt_private * prv = RT_PRIV(ops);
    spinlock_t *lock = vcpu_schedule_lock_irq(vc);

    clear_bit(__RT_scheduled, &svc->flags);
    /* not insert idle vcpu to runq */
    if ( is_idle_vcpu(vc) ) 
        goto out;

    if ( test_and_clear_bit(__RT_delayed_runq_add, &svc->flags) && 
         likely(vcpu_runnable(vc)) ) 
    {
        __runq_insert(ops, svc);
        __repl_update(ops, NOW());
        snext = __runq_pick(ops, prv->cpus);    /* pick snext from ALL cpus */
        runq_tickle(ops, snext);
    }
out:
    vcpu_schedule_unlock_irq(lock, vc);
}

/*
 * set/get each vcpu info of each domain
 */
static int
rt_dom_cntl(
    const struct scheduler *ops, 
    struct domain *d, 
    struct xen_domctl_scheduler_op *op)
{
    struct rt_dom * const sdom = RT_DOM(d);
    struct rt_vcpu * svc;
    struct list_head *iter;
    int rc = 0;

    switch ( op->cmd )
    {
    case XEN_DOMCTL_SCHEDOP_getinfo:
        /* for debug use, whenever adjust Dom0 parameter, do global dump */
        if ( d->domain_id == 0 ) 
            rt_dump(ops);

        svc = list_entry(sdom->vcpu.next, struct rt_vcpu, sdom_elem);
        op->u.rt.period = svc->period / MICROSECS(1); /* transfer to us */
        op->u.rt.budget = svc->budget / MICROSECS(1);
        break;
    case XEN_DOMCTL_SCHEDOP_putinfo:
        list_for_each( iter, &sdom->vcpu ) 
        {
            struct rt_vcpu * svc = list_entry(iter, struct rt_vcpu, sdom_elem);
            svc->period = MICROSECS(op->u.rt.period); /* transfer to nanosec */
            svc->budget = MICROSECS(op->u.rt.budget);
        }
        break;
    }

    return rc;
}

static struct rt_private _rt_priv;

const struct scheduler sched_rt_ds_def = {
    .name           = "SMP RT DS Scheduler",
    .opt_name       = "rt_ds",
    .sched_id       = XEN_SCHEDULER_RT_DS,
    .sched_data     = &_rt_priv,

    .dump_cpu_state = rt_dump_pcpu,
    .dump_settings  = rt_dump,
    .init           = rt_init,
    .deinit         = rt_deinit,
    .alloc_pdata    = rt_alloc_pdata,
    .free_pdata     = rt_free_pdata,
    .alloc_domdata  = rt_alloc_domdata,
    .free_domdata   = rt_free_domdata,
    .init_domain    = rt_dom_init,
    .destroy_domain = rt_dom_destroy,
    .alloc_vdata    = rt_alloc_vdata,
    .free_vdata     = rt_free_vdata,
    .insert_vcpu    = rt_vcpu_insert,
    .remove_vcpu    = rt_vcpu_remove,

    .adjust         = rt_dom_cntl,

    .pick_cpu       = rt_cpu_pick,
    .do_schedule    = rt_schedule,
    .sleep          = rt_vcpu_sleep,
    .wake           = rt_vcpu_wake,
    .context_saved  = rt_context_saved,
};
