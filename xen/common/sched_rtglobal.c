
/******************************************************************************
 * Preemptive Global EDF/RM scheduler for xen
 *
 * by Sisu Xi, 2013, Washington University in Saint Louis
 * and Meng Xu, 2014, University of Pennsylvania
 *
 * based on the code of credite Scheduler
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

/*
 * TODO:
 *
 * How to show individual VCPU info?
 * Migration compensation and resist like credit2
 * Lock Holder Problem, using yield?
 * Self switch problem?
 * More testing with xentrace and xenanalyze
 */

/*
 * Design:
 *
 * Follows the pre-emptive Global EDF/RM theory.
 * Each VCPU can have a dedicated period/budget pair of parameter. When a VCPU is running, it burns its budget, and when there are no budget, the VCPU need to wait unitl next period to get replensihed. Any unused budget is discarded in the end of period.
 * Server mechanism: deferrable server is used here. Therefore, when a VCPU has no task but with budget left, the budget is preserved.
 * Priority scheme: a global variable called priority_scheme is used to switch between EDF and RM
 * Queue scheme: a global runqueue is used here. It holds all runnable VCPUs. VCPUs are divided into two parts: with and without remaining budget. Among each part, VCPUs are sorted by their current deadlines.
 * Scheduling quanta: 1 ms is picked as the scheduling quanta, but the accounting is done in microsecond level.
 * Other: cpumask is also supported, as a result, although the runq is sorted, the scheduler also need to verify whether the cpumask is allowed or not.
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

#define EDF							0
#define RM							1

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
    long cur_budget;             /* current budget in microseconds, if none, should not schedule unless extra */
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
    int    extra;               /* not evaluated */
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
 * EDF schedule policy: vcpu with smaller deadline has higher priority
 * RM schedule policy: vcpu with smaller period has higher priority
 * Lock is grabbed before calling this function */
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

		if ( svc->cur_budget > 0 ) { // svc still has budget
			if ( iter_svc->cur_budget == 0 ||
			     ( ( prv->priority_scheme == EDF && svc->cur_deadline <= iter_svc->cur_deadline ) ||
			       ( prv->priority_scheme == RM && svc->period <= iter_svc->period )) ) {
					break;
			}
		} else { // svc has no budget
			if ( iter_svc->cur_budget == 0 &&
			     ( ( prv->priority_scheme == EDF && svc->cur_deadline <= iter_svc->cur_deadline ) ||
			       ( prv->priority_scheme == RM && svc->period <= iter_svc->period )) ) {
					break;
			}
		}
    }

    list_add_tail(&svc->runq_elem, iter);
}


/**
 * Debug related code, dump vcpu/cpu information
 */
static void
rtglobal_dump_vcpu(struct rtglobal_vcpu *svc)
{
    if ( svc == NULL ) {
        printk("NULL!\n");
        return;
    }
    printk("[%5d.%-2d] cpu %d, (%-2d, %-2d), cur_b=%ld cur_d=%lu last_start=%lu onR=%d runnable=%d\n",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->vcpu->processor,
            svc->period,
            svc->budget,
            svc->cur_budget,
            svc->cur_deadline,
            svc->last_start,
            __vcpu_on_runq(svc),
            vcpu_runnable(svc->vcpu));
}

static void
rtglobal_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct rtglobal_vcpu *svc = RTGLOBAL_VCPU(curr_on_cpu(cpu));

    printtime();
    rtglobal_dump_vcpu(svc);
}

/* should not need lock here. only showing stuff */
static void
rtglobal_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom, *iter_svc, *runq, *iter;
    struct rtglobal_private *prv = RTGLOBAL_PRIV(ops);
    struct rtglobal_vcpu *svc;
    int cpu = 0;
    int loop = 0;

    printtime();
    printk("Priority Scheme: ");
    if ( prv->priority_scheme == EDF ) printk("EDF\n");
    else printk ("RM\n");

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

    printk("This is the Deferrable Server version of the preemptive RTGLOBAL scheduler\n");
    printk("If you want to use it as a periodic server, please run a background busy CPU task\n");
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

/* point per_cpu spinlock to the global system lock; all cpu have same global system lock */
static void *
rtglobal_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct rtglobal_private *prv = RTGLOBAL_PRIV(ops);

    cpumask_set_cpu(cpu, &prv->cpus);

    per_cpu(schedule_data, cpu).schedule_lock = &prv->lock;

    printtime();
    printk("total cpus: %d", cpumask_weight(&prv->cpus));
    return (void *)1;
}

static void
rtglobal_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct rtglobal_private * prv = RTGLOBAL_PRIV(ops);
    cpumask_clear_cpu(cpu, &prv->cpus);
    printtime();
    printk("cpu=%d\n", cpu);
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
    sdom->extra = 0;         /* by default, do not allow extra time */

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

/* Add vcpu to its domain's vcpu list;
 * Lock is grabbed before calling this function */
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

/* Remove vcpu from its domain's vcpu list
 * Lock is grabbed before calling this function */
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

/* Implemented as deferrable server. 
 * Different server mechanism has different implementation.
 * burn budget at microsecond level. */
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
