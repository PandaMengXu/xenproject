/******************************************************************************
 * Preemptive Global Earliest Deadline First  (EDF) scheduler for Xen
 * EDF scheduling is one of most popular real-time scheduling algorithm used in
 * embedded field.
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
 * This scheduler follows the pre-emptive Global EDF theory in real-time field.
 * Each VCPU can have a dedicated period/budget pair of parameter. 
 * A VCPU has its budget replenished at the beginning of each of its periods;
 * The VCPU discards its unused budget at the end of each of its periods.
 * If a VCPU runs out of budget in a period, it has to wait until next period.
 * The mechanism of how to burn a VCPU's budget depends on the server mechanism
 * implemented for each VCPU.
 * The mechanism of deciding the priority of VCPUs at each scheduling point
 * depends on the priority scheme.
 *
 * Server mechanism: a VCPU is implemented as a deferable server.
 * When a VCPU has a task running on it, its budget is continuously burned;
 * When a VCPU has no task but with budget left, its budget is preserved.
 *
 * Priority scheme: Earliest Deadline First (EDF).
 * At any scheduling point, the VCPU with earliest deadline has highest priority.
 *
 * Queue scheme: A global runqueue for each CPU pool. 
 * The runqueue holds all runnable VCPUs. 
 * VCPUs in the runqueue are divided into two parts: with and without remaining budget. 
 * At each part, VCPUs are sorted based on EDF priority scheme.
 *
 * Scheduling quanta: 1 ms; but accounting the budget is in microsecond.
 *
 * Note: cpumask and cpupool is supported.
 */

/*
 * Locking:
 * Just like credit2, a global system lock is used to protect the RunQ.
 *
 * The lock is already grabbed when calling wake/sleep/schedule/ functions in schedule.c
 *
 * The functions involes RunQ and needs to grab locks are:
 *    dump, vcpu_insert, vcpu_remove, context_saved,
 */


/*
 * Default parameters
 */
#define RTGLOBAL_DEFAULT_PERIOD     10
#define RTGLOBAL_DEFAULT_BUDGET      4

#define EDF							81

/*
 * Useful macros
 */
#define RTGLOBAL_PRIV(_ops)     ((struct rtglobal_private *)((_ops)->sched_data))
#define RTGLOBAL_VCPU(_vcpu)    ((struct rtglobal_vcpu *)(_vcpu)->sched_priv)
#define RTGLOBAL_DOM(_dom)      ((struct rtglobal_dom *)(_dom)->sched_priv)
#define RUNQ(_ops)          	(&RTGLOBAL_PRIV(_ops)->runq)

/*
 * Flags
 */
#define __RTGLOBAL_scheduled            1
#define RTGLOBAL_scheduled (1<<__RTGLOBAL_scheduled)
#define __RTGLOBAL_delayed_runq_add     2
#define RTGLOBAL_delayed_runq_add (1<<__RTGLOBAL_delayed_runq_add)

/*
 * Used to printout debug information
 */
#define printtime()     ( printk("%d : %3ld.%3ld : %-19s ", smp_processor_id(), NOW()/MILLISECS(1), NOW()%MILLISECS(1)/1000, __func__) )

/*
 * Systme-wide private data, include a global RunQueue
 * The global lock is referenced by schedule_data.schedule_lock from all physical cpus.
 * It can be grabbed via vcpu_schedule_lock_irq()
 */
struct rtglobal_private {
    spinlock_t lock;        /* The global coarse grand lock */
    struct list_head sdom;  /* list of availalbe domains, used for dump */
    struct list_head runq;  /* Ordered list of runnable VMs */
    cpumask_t cpus;         /* cpumask_t of available physical cpus */
    cpumask_t tickled;      /* another cpu in the queue already ticked this one */
    unsigned priority_scheme;
};

/*
 * Virtual CPU
 */
struct rtglobal_vcpu {
    struct list_head runq_elem; /* On the runqueue list */
    struct list_head sdom_elem; /* On the domain VCPU list */

    /* Up-pointers */
    struct rtglobal_dom *sdom;
    struct vcpu *vcpu;

    /* VCPU parameters, in milliseconds */
    int period;
    int budget;

    /* VCPU current infomation */
    long cur_budget;             /* current budget in microseconds */
    s_time_t last_start;        /* last start time, used to calculate budget */
    s_time_t cur_deadline;      /* current deadline, used to do EDF */
    unsigned flags;             /* mark __RTGLOBAL_scheduled, etc.. */
};

/*
 * Domain
 */
struct rtglobal_dom {
    struct list_head vcpu;      /* link its VCPUs */
    struct list_head sdom_elem; /* link list on rtglobal_priv */
    struct domain *dom;         /* pointer to upper domain */
};

/*
 * RunQueue helper functions
 */
static int
__vcpu_on_runq(struct rtglobal_vcpu *svc)
{
   return !list_empty(&svc->runq_elem);
}

static struct rtglobal_vcpu *
__runq_elem(struct list_head *elem)
{
    return list_entry(elem, struct rtglobal_vcpu, runq_elem);
}

/* lock is grabbed before calling this function */
static inline void
__runq_remove(struct rtglobal_vcpu *svc)
{
    if ( __vcpu_on_runq(svc) )
        list_del_init(&svc->runq_elem);
}

/* Insert vcpu into runq based on vcpu's priority
 * EDF schedule policy: vcpu with smaller deadline has higher priority;
 * When vcpus same periods, insert the current one at the head of these vcpus.
 * Lock is grabbed before calling this function 
 */
static void
__runq_insert(const struct scheduler *ops, struct rtglobal_vcpu *svc)
{
    struct list_head *runq = RUNQ(ops);
    struct list_head *iter;
	struct rtglobal_private *prv = RTGLOBAL_PRIV(ops);
    ASSERT( spin_is_locked(per_cpu(schedule_data, svc->vcpu->processor).schedule_lock) );

    if ( __vcpu_on_runq(svc) )
        return;

    list_for_each(iter, runq) {
        struct rtglobal_vcpu * iter_svc = __runq_elem(iter);

		if ( svc->cur_budget > 0 ) { /* svc still has budget */
			if ( iter_svc->cur_budget == 0 ||
			     ( prv->priority_scheme == EDF && svc->cur_deadline <= iter_svc->cur_deadline ) ) {
					break;
			}
		} else { /* svc has no budget */
			if ( iter_svc->cur_budget == 0 &&
			     ( prv->priority_scheme == EDF && svc->cur_deadline <= iter_svc->cur_deadline ) ) {
					break;
			}
		}
    }

    list_add_tail(&svc->runq_elem, iter);
}


/*
 * Debug related code, dump vcpu/cpu information
 */
static void
rtglobal_dump_vcpu(struct rtglobal_vcpu *svc)
{
    if ( svc == NULL ) {
        printk("NULL!\n");
        return;
    }
#define cpustr keyhandler_scratch
    cpumask_scnprintf(cpustr, sizeof(cpustr), svc->vcpu->cpu_hard_affinity);
    printk("[%5d.%-2d] cpu %d, (%-2d, %-2d), cur_b=%ld cur_d=%lu last_start=%lu onR=%d runnable=%d cpu_hard_affinity=%s ",
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
    memset(cpustr, 0, sizeof(char)*1024);
    cpumask_scnprintf(cpustr, sizeof(cpustr), cpupool_scheduler_cpumask(svc->vcpu->domain->cpupool));
    printk("cpupool=%s\n", cpustr);
#undef cpustr
}

static void
rtglobal_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct rtglobal_vcpu *svc = RTGLOBAL_VCPU(curr_on_cpu(cpu));

    printtime();
    rtglobal_dump_vcpu(svc);
}

/*
 * should not need lock here. only showing stuff 
 */
static void
rtglobal_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom, *iter_svc, *runq, *iter;
    struct rtglobal_private *prv = RTGLOBAL_PRIV(ops);
    struct rtglobal_vcpu *svc;
    int cpu = 0;
    int loop = 0;

    printtime();
    printk("Priority Scheme: EDF\n");

    printk("PCPU info: \n");
    for_each_cpu(cpu, &prv->cpus) {
        rtglobal_dump_pcpu(ops, cpu);
    }

    printk("Global RunQueue info: \n");
    loop = 0;
    runq = RUNQ(ops);
    list_for_each( iter, runq ) {
        svc = __runq_elem(iter);
        printk("\t%3d: ", ++loop);
        rtglobal_dump_vcpu(svc);
    }

    printk("Domain info: \n");
    loop = 0;
    list_for_each( iter_sdom, &prv->sdom ) {
        struct rtglobal_dom *sdom;
        sdom = list_entry(iter_sdom, struct rtglobal_dom, sdom_elem);
        printk("\tdomain: %d\n", sdom->dom->domain_id);

        list_for_each( iter_svc, &sdom->vcpu ) {
            svc = list_entry(iter_svc, struct rtglobal_vcpu, sdom_elem);
            printk("\t\t%3d: ", ++loop);
            rtglobal_dump_vcpu(svc);
        }
    }

    printk("\n");
}

/*
 * Init/Free related code
 */
static int
rtglobal_init(struct scheduler *ops)
{
    struct rtglobal_private *prv;

    prv = xmalloc(struct rtglobal_private);
    if ( prv == NULL ) {
        printk("malloc failed at rtglobal_private\n");
        return -ENOMEM;
    }
    memset(prv, 0, sizeof(*prv));

    ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->sdom);
    INIT_LIST_HEAD(&prv->runq);
    cpumask_clear(&prv->tickled);
    cpumask_clear(&prv->cpus);
    prv->priority_scheme = EDF;     /* by default, use EDF scheduler */

    printtime();
    printk("\n");

    return 0;
}

static void
rtglobal_deinit(const struct scheduler *ops)
{
    struct rtglobal_private *prv;

    printtime();
    printk("\n");

    prv = RTGLOBAL_PRIV(ops);
    if ( prv )
        xfree(prv);
}

/* 
 * point per_cpu spinlock to the global system lock; all cpu have same global system lock 
 */
static void *
rtglobal_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct rtglobal_private *prv = RTGLOBAL_PRIV(ops);

    cpumask_set_cpu(cpu, &prv->cpus);

    per_cpu(schedule_data, cpu).schedule_lock = &prv->lock;

    printtime();
    printk("%s total cpus: %d", __FUNCTION__, cpumask_weight(&prv->cpus));
    return (void *)1;
}

static void
rtglobal_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);
    cpumask_clear_cpu(cpu, &prv->cpus);
    printtime();
    printk("%s cpu=%d\n", __FUNCTION__, cpu);
}

static void *
rtglobal_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
    unsigned long flags;
    struct rtglobal_dom *sdom;
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);

    printtime();
    printk("dom=%d\n", dom->domain_id);

    sdom = xmalloc(struct rtglobal_dom);
    if ( sdom == NULL ) {
        printk("%s, xmalloc failed\n", __func__);
        return NULL;
    }
    memset(sdom, 0, sizeof(*sdom));

    INIT_LIST_HEAD(&sdom->vcpu);
    INIT_LIST_HEAD(&sdom->sdom_elem);
    sdom->dom = dom;

    /* spinlock here to insert the dom */
    spin_lock_irqsave(&prv->lock, flags);
    list_add_tail(&sdom->sdom_elem, &(prv->sdom));
    spin_unlock_irqrestore(&prv->lock, flags);

    return (void *)sdom;
}

static void
rtglobal_free_domdata(const struct scheduler *ops, void *data)
{
    unsigned long flags;
    struct rtglobal_dom *sdom = data;
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);

    printtime();
    printk("dom=%d\n", sdom->dom->domain_id);

    spin_lock_irqsave(&prv->lock, flags);
    list_del_init(&sdom->sdom_elem);
    spin_unlock_irqrestore(&prv->lock, flags);
    xfree(data);
}

static int
rtglobal_dom_init(const struct scheduler *ops, struct domain *dom)
{
    struct rtglobal_dom *sdom;

    printtime();
    printk("dom=%d\n", dom->domain_id);

    /* IDLE Domain does not link on rtglobal_private */
    if ( is_idle_domain(dom) ) { return 0; }

    sdom = rtglobal_alloc_domdata(ops, dom);
    if ( sdom == NULL ) {
        printk("%s, failed\n", __func__);
        return -ENOMEM;
    }
    dom->sched_priv = sdom;

    return 0;
}

static void
rtglobal_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
    printtime();
    printk("dom=%d\n", dom->domain_id);

    rtglobal_free_domdata(ops, RTGLOBAL_DOM(dom));
}

static void *
rtglobal_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct rtglobal_vcpu *svc;
    s_time_t now = NOW();
    long count;

    /* Allocate per-VCPU info */
    svc = xmalloc(struct rtglobal_vcpu);
    if ( svc == NULL ) {
        printk("%s, xmalloc failed\n", __func__);
        return NULL;
    }
    memset(svc, 0, sizeof(*svc));

    INIT_LIST_HEAD(&svc->runq_elem);
    INIT_LIST_HEAD(&svc->sdom_elem);
    svc->flags = 0U;
    svc->sdom = dd;
    svc->vcpu = vc;
    svc->last_start = 0;            /* init last_start is 0 */

	svc->period = RTGLOBAL_DEFAULT_PERIOD;
	if ( !is_idle_vcpu(vc) && vc->domain->domain_id != 0 ) {
		svc->budget = RTGLOBAL_DEFAULT_BUDGET;
	} else {
		svc->budget = RTGLOBAL_DEFAULT_PERIOD;
	}

	count = (now/MILLISECS(svc->period)) + 1;
	svc->cur_deadline += count*MILLISECS(svc->period); /* sync all VCPU's start time to 0 */

    svc->cur_budget = svc->budget*1000; /* counting in microseconds level */
    printtime();
    rtglobal_dump_vcpu(svc);

    return svc;
}

static void
rtglobal_free_vdata(const struct scheduler *ops, void *priv)
{
    struct rtglobal_vcpu *svc = priv;
    printtime();
    rtglobal_dump_vcpu(svc);
    xfree(svc);
}

/* 
 * Add vcpu to its domain's vcpu list;
 * Lock is grabbed before calling this function 
 */
static void
rtglobal_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtglobal_vcpu *svc = RTGLOBAL_VCPU(vc);

    printtime();
    rtglobal_dump_vcpu(svc);

    /* IDLE VCPU not allowed on RunQ */
    if ( is_idle_vcpu(vc) )
        return;

    list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);   /* add to dom vcpu list */
}

/*
 * Remove vcpu from its domain's vcpu list
 * Lock is grabbed before calling this function 
 */
static void
rtglobal_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtglobal_vcpu * const svc = RTGLOBAL_VCPU(vc);
    struct rtglobal_dom * const sdom = svc->sdom;

    printtime();
    rtglobal_dump_vcpu(svc);

    BUG_ON( sdom == NULL );
    BUG_ON( __vcpu_on_runq(svc) );

    if ( !is_idle_vcpu(vc) ) {
        list_del_init(&svc->sdom_elem);
    }
}

/* 
 * Pick a valid CPU for the vcpu vc
 * Valid CPU of a vcpu is intesection of vcpu's affinity and available cpus
 */
static int
rtglobal_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    cpumask_t cpus;
    cpumask_t *online;
    int cpu;
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);

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
 * Implemented as deferrable server. 
 * Different server mechanism has different implementation.
 * burn budget at microsecond level. 
 */
static void
burn_budgets(const struct scheduler *ops, struct rtglobal_vcpu *svc, s_time_t now) {
    s_time_t delta;
    unsigned int consume;
    long count = 0;

    /* first time called for this svc, update last_start */
    if ( svc->last_start == 0 ) {
        svc->last_start = now;
        return;
    }

    /* don't burn budget for idle VCPU */
    if ( is_idle_vcpu(svc->vcpu) ) {
        return;
    }

    /* don't burn budget for Domain-0, RT-Xen use only */
    if ( svc->sdom->dom->domain_id == 0 ) {
        return;
    }

    /* update deadline info */
    delta = now - svc->cur_deadline;
    if ( delta >= 0 ) {
        count = ( delta/MILLISECS(svc->period) ) + 1;
        svc->cur_deadline += count * MILLISECS(svc->period);
        svc->cur_budget = svc->budget * 1000;
        return;
    }

    delta = now - svc->last_start;
    if ( delta < 0 ) {
        printk("%s, delta = %ld for ", __func__, delta);
        rtglobal_dump_vcpu(svc);
        svc->last_start = now;  /* update last_start */
        svc->cur_budget = 0;
        return;
    }

    if ( svc->cur_budget == 0 ) return;

    /* burn at microseconds level */
    consume = ( delta/MICROSECS(1) );
    if ( delta%MICROSECS(1) > MICROSECS(1)/2 ) consume++;

    svc->cur_budget -= consume;
    if ( svc->cur_budget < 0 ) svc->cur_budget = 0;
}

/* 
 * RunQ is sorted. Pick first one within cpumask. If no one, return NULL
 * lock is grabbed before calling this function 
 */
static struct rtglobal_vcpu *
__runq_pick(const struct scheduler *ops, cpumask_t mask)
{
    struct list_head *runq = RUNQ(ops);
    struct list_head *iter;
    struct rtglobal_vcpu *svc = NULL;
    struct rtglobal_vcpu *iter_svc = NULL;
    cpumask_t cpu_common;
    cpumask_t *online;
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);

    list_for_each(iter, runq) {
        iter_svc = __runq_elem(iter);

        /* mask is intersection of cpu_hard_affinity and cpupool and priv->cpus */
        online = cpupool_scheduler_cpumask(iter_svc->vcpu->domain->cpupool);
        cpumask_and(&cpu_common, online, &prv->cpus);
        cpumask_and(&cpu_common, &cpu_common, iter_svc->vcpu->cpu_hard_affinity);
        cpumask_and(&cpu_common, &mask, &cpu_common);
        if ( cpumask_empty(&cpu_common) )
            continue;

        if ( iter_svc->cur_budget <= 0 )
            continue;

        svc = iter_svc;
        break;
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
    struct rtglobal_vcpu *svc = NULL;

    s_time_t diff;
    long count;

    list_for_each_safe(iter, tmp, runq) {
        svc = __runq_elem(iter);

        diff = now - svc->cur_deadline;
        if ( diff > 0 ) {
            count = (diff/MILLISECS(svc->period)) + 1;
            svc->cur_deadline += count * MILLISECS(svc->period);
            svc->cur_budget = svc->budget * 1000;
            __runq_remove(svc);
            __runq_insert(ops, svc);
        }
    }
}

/* 
 * schedule function for rtglobal scheduler.
 * The lock is already grabbed in schedule.c, no need to lock here 
 */
static struct task_slice
rtglobal_schedule(const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    const int cpu = smp_processor_id();
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);
    struct rtglobal_vcpu * const scurr = RTGLOBAL_VCPU(current);
    struct rtglobal_vcpu * snext = NULL;
    struct task_slice ret;

    /* clear ticked bit now that we've been scheduled */
    if ( cpumask_test_cpu(cpu, &prv->tickled) )
        cpumask_clear_cpu(cpu, &prv->tickled);

    /* burn_budget would return for IDLE VCPU */
    burn_budgets(ops, scurr, now);

    __repl_update(ops, now);

    if ( tasklet_work_scheduled ) {
        snext = RTGLOBAL_VCPU(idle_vcpu[cpu]);
    } else {
        cpumask_t cur_cpu;
        cpumask_clear(&cur_cpu);
        cpumask_set_cpu(cpu, &cur_cpu);
        snext = __runq_pick(ops, cur_cpu);
        if ( snext == NULL )
            snext = RTGLOBAL_VCPU(idle_vcpu[cpu]);

        /* if scurr has higher priority and budget, still pick scurr */
        if ( !is_idle_vcpu(current) &&
             vcpu_runnable(current) &&
             scurr->cur_budget > 0 &&
             ( is_idle_vcpu(snext->vcpu) ||
               ( prv->priority_scheme == EDF && 
                scurr->cur_deadline <= snext->cur_deadline ) ) ) {
            snext = scurr;
        }
    }

    if ( snext != scurr &&
         !is_idle_vcpu(current) &&
         vcpu_runnable(current) ) {
        set_bit(__RTGLOBAL_delayed_runq_add, &scurr->flags);
    }

    snext->last_start = now;
    ret.migrated = 0;
    if ( !is_idle_vcpu(snext->vcpu) ) {
        if ( snext != scurr ) {
            __runq_remove(snext);
            set_bit(__RTGLOBAL_scheduled, &snext->flags);
        }
        if ( snext->vcpu->processor != cpu ) {
            snext->vcpu->processor = cpu;
            ret.migrated = 1;
        }
    }

    ret.time = MILLISECS(1);
    ret.task = snext->vcpu;

    return ret;
}

/*
 * Remove VCPU from RunQ
 * The lock is already grabbed in schedule.c, no need to lock here 
 */
static void
rtglobal_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtglobal_vcpu * const svc = RTGLOBAL_VCPU(vc);

    BUG_ON( is_idle_vcpu(vc) );

    if ( curr_on_cpu(vc->processor) == vc ) {
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);
        return;
    }

    if ( __vcpu_on_runq(svc) ) {
        __runq_remove(svc);
    }

    clear_bit(__RTGLOBAL_delayed_runq_add, &svc->flags);
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
runq_tickle(const struct scheduler *ops, struct rtglobal_vcpu *new)
{
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);
    struct rtglobal_vcpu * scheduled = NULL;    /* lowest priority scheduled */
    struct rtglobal_vcpu * iter_svc;
    struct vcpu * iter_vc;
    int cpu = 0;
    cpumask_t not_tickled;                      /* not tickled cpus */
    cpumask_t *online;

    if ( new == NULL || is_idle_vcpu(new->vcpu) ) return;

    online = cpupool_scheduler_cpumask(new->vcpu->domain->cpupool);
    cpumask_and(&not_tickled, online, &prv->cpus);
    cpumask_and(&not_tickled, &not_tickled, new->vcpu->cpu_hard_affinity);
    cpumask_andnot(&not_tickled, &not_tickled, &prv->tickled);

    /* 1) if new's previous cpu is idle, kick it for cache benefit */
    if ( is_idle_vcpu(curr_on_cpu(new->vcpu->processor)) ) {
        cpumask_set_cpu(new->vcpu->processor, &prv->tickled);
        cpu_raise_softirq(new->vcpu->processor, SCHEDULE_SOFTIRQ);
        return;
    }

    /* 2) if there are any idle pcpu, kick it */
    /* The same loop also find the one with lowest priority */
    for_each_cpu(cpu, &not_tickled) {
        iter_vc = curr_on_cpu(cpu);
        if ( is_idle_vcpu(iter_vc) ) {
            cpumask_set_cpu(cpu, &prv->tickled);
            cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
            return;
        }
        iter_svc = RTGLOBAL_VCPU(iter_vc);
        if ( scheduled == NULL || 
            ( prv->priority_scheme == EDF &&
              iter_svc->cur_deadline > scheduled->cur_deadline) ) {
            scheduled = iter_svc;
        }
    }

    /* 3) candicate has higher priority, kick out the lowest priority vcpu */
    if ( scheduled != NULL && new->cur_deadline < scheduled->cur_deadline ) {
        cpumask_set_cpu(scheduled->vcpu->processor, &prv->tickled);
        cpu_raise_softirq(scheduled->vcpu->processor, SCHEDULE_SOFTIRQ);
    }
    return;
}

/* 
 * Should always wake up runnable vcpu, put it back to RunQ. 
 * Check priority to raise interrupt 
 * The lock is already grabbed in schedule.c, no need to lock here 
 * TODO: what if these two vcpus belongs to the same domain? 
 */
static void
rtglobal_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtglobal_vcpu * const svc = RTGLOBAL_VCPU(vc);
    s_time_t diff;
    s_time_t now = NOW();
    long count = 0;
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);
    struct rtglobal_vcpu * snext = NULL;        /* highest priority on RunQ */

    BUG_ON( is_idle_vcpu(vc) );

    if ( unlikely(curr_on_cpu(vc->processor) == vc) ) return;

    /* on RunQ, just update info is ok */
    if ( unlikely(__vcpu_on_runq(svc)) ) return;

    /* If context hasn't been saved yet, set flag so it will add later */
    if ( unlikely(test_bit(__RTGLOBAL_scheduled, &svc->flags)) ) {
        set_bit(__RTGLOBAL_delayed_runq_add, &svc->flags);
        return;
    }

    /* update deadline info */
    diff = now - svc->cur_deadline;
    if ( diff >= 0 ) {
        count = ( diff/MILLISECS(svc->period) ) + 1;
        svc->cur_deadline += count * MILLISECS(svc->period);
        svc->cur_budget = svc->budget * 1000;
    }

    __runq_insert(ops, svc);
    __repl_update(ops, now);
    snext = __runq_pick(ops, prv->cpus);    /* pick snext from ALL valid cpus */
    runq_tickle(ops, snext);

    return;
}

/* scurr has finished context switch, insert it back to the RunQ,
 * and then pick the highest priority vcpu from runq to run 
 */
static void
rtglobal_context_saved(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtglobal_vcpu * svc = RTGLOBAL_VCPU(vc);
    struct rtglobal_vcpu * snext = NULL;
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);
    spinlock_t *lock;

    clear_bit(__RTGLOBAL_scheduled, &svc->flags);
    if ( is_idle_vcpu(vc) ) return;

    lock = vcpu_schedule_lock_irq(vc);
    if ( test_and_clear_bit(__RTGLOBAL_delayed_runq_add, &svc->flags) && 
         likely(vcpu_runnable(vc)) ) {
        __runq_insert(ops, svc);
        __repl_update(ops, NOW());
        snext = __runq_pick(ops, prv->cpus);    /* pick snext from ALL cpus */
        runq_tickle(ops, snext);
    }
    vcpu_schedule_unlock_irq(lock, vc);
}

/*
 * set/get each vcpu info of each domain
 */
static int
rtglobal_dom_cntl(const struct scheduler *ops, struct domain *d, struct xen_domctl_scheduler_op *op)
{
    xen_domctl_sched_rtglobal_params_t local_sched;
    struct rtglobal_dom * const sdom = RTGLOBAL_DOM(d);
    struct list_head *iter;
    int vcpu_index = 0;
    int rc = -EINVAL;

    switch ( op->cmd )
    {
    case XEN_DOMCTL_SCHEDOP_getinfo:
        /* for debug use, whenever adjust Dom0 parameter, do global dump */
        if ( d->domain_id == 0 ) {
            rtglobal_dump(ops);
        }

        local_sched.num_vcpus = 0;
        list_for_each( iter, &sdom->vcpu ) {
            struct rtglobal_vcpu * svc = list_entry(iter, struct rtglobal_vcpu, sdom_elem);

            ASSERT(vcpu_index < XEN_LEGACY_MAX_VCPUS);
            local_sched.vcpus[vcpu_index].budget = svc->budget;
            local_sched.vcpus[vcpu_index].period = svc->period;

            vcpu_index++;
        }
        local_sched.num_vcpus = vcpu_index;
        copy_to_guest(op->u.rtglobal.schedule, &local_sched, 1);
        rc = 0;
        break;
    case XEN_DOMCTL_SCHEDOP_putinfo:
        copy_from_guest(&local_sched, op->u.rtglobal.schedule, 1);
        list_for_each( iter, &sdom->vcpu ) {
            struct rtglobal_vcpu * svc = list_entry(iter, struct rtglobal_vcpu, sdom_elem);

            if ( local_sched.vcpu_index == svc->vcpu->vcpu_id ) { /* adjust per VCPU parameter */
                vcpu_index = local_sched.vcpu_index;

                if ( vcpu_index < 0 || vcpu_index > XEN_LEGACY_MAX_VCPUS) {
                    printk("XEN_DOMCTL_SCHEDOP_putinfo: vcpu_index=%d\n",
                            vcpu_index);
                }else{
                    printk("XEN_DOMCTL_SCHEDOP_putinfo: vcpu_index=%d, period=%d, budget=%d\n",
                            vcpu_index, local_sched.vcpus[vcpu_index].period, 
                            local_sched.vcpus[vcpu_index].budget);
                }

                svc->period = local_sched.vcpus[vcpu_index].period;
                svc->budget = local_sched.vcpus[vcpu_index].budget;

                break;
            }
        }
        rc = 0;
        break;
    }

    return rc;
}

static struct rtglobal_private _rtglobal_priv;

const struct scheduler sched_rtglobal_def = {
    .name           = "SMP RTGLOBAL Scheduler",
    .opt_name       = "rtglobal",
    .sched_id       = XEN_SCHEDULER_RTGLOBAL,
    .sched_data     = &_rtglobal_priv,

    .dump_cpu_state = rtglobal_dump_pcpu,
    .dump_settings  = rtglobal_dump,
    .init           = rtglobal_init,
    .deinit         = rtglobal_deinit,
    .alloc_pdata    = rtglobal_alloc_pdata,
    .free_pdata     = rtglobal_free_pdata,
    .alloc_domdata  = rtglobal_alloc_domdata,
    .free_domdata   = rtglobal_free_domdata,
    .init_domain    = rtglobal_dom_init,
    .destroy_domain = rtglobal_dom_destroy,
    .alloc_vdata    = rtglobal_alloc_vdata,
    .free_vdata     = rtglobal_free_vdata,
    .insert_vcpu    = rtglobal_vcpu_insert,
    .remove_vcpu    = rtglobal_vcpu_remove,

    .adjust         = rtglobal_dom_cntl,

    .pick_cpu       = rtglobal_cpu_pick,
    .do_schedule    = rtglobal_schedule,
    .sleep          = rtglobal_vcpu_sleep,
    .wake           = rtglobal_vcpu_wake,
    .context_saved  = rtglobal_context_saved,

    .yield          = NULL,
    .migrate        = NULL,
};
