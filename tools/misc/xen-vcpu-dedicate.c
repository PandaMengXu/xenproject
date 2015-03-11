#include <xenctrl.h>
#include <xc_private.h>
#include <xc_core.h>
#include <unistd.h>
#include <inttypes.h>
#include <assert.h>

#include "xg_save_restore.h"

#undef ARRAY_SIZE /* We shouldn't be including xc_private.h */
#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))

static xc_interface *xch;

int help_func(int argc, char *argv[])
{
    fprintf(stderr,
            "Usage: xen-vcpu-dedicate <command> [args]\n"
            "Commands:\n"
            "  help                        show this help\n"
            "  add  <dom_id> <vcpu_id>\n"
                    "set vcpu <vcpu_id> of dom <domid> as dedicated vcpu\n"
            "  rm  <dom_id> <vcpu_id>"
                    "remove vcpu <vcpu_id> of dom <domid> as dedicated vcpu\n"
           );

    return 0;
}

int add_func(int argc, char *argv[])
{
    int vcpu_id, dom_id;
    struct xen_domctl_sched_rtds sdom;

    if ( argc < 2 )
    {
        help_func(0, NULL);
        return 1;
    }

    dom_id = atoi(argv[0]);
    vcpu_id = atoi(argv[1]);
    /* TODO: sanity check dom_id and vcpu_id is valid */
    if ( dom_id < 0 || vcpu_id < 0 )
    {
        ERROR("both dom_id (%d) and vcpu_id (%d) must be non-negative\n",
               dom_id, vcpu_id);
    }

    sdom.vcpuid = vcpu_id;
    if ( xc_sched_rtds_domain_set_dedvcpu(xch, dom_id, &sdom) )
    {
        ERROR("Could not set vcpu %d of dom %d as dedicated VCPU\n",
              vcpu_id, dom_id);
        return -1;
    }

    printf("vcpu %d of dom %d has been set as dedicated VCPU\n",
           vcpu_id, dom_id);
    return 0;
}

int rm_func(int argc, char *argv[])
{
    printf("TO IMPLEMENT\n");
    return 0;
}


struct {
    const char *name;
    int (*func)(int argc, char *argv[]);
} opts[] = {
    { "help", help_func },
    { "add", add_func },
    { "rm", rm_func },
};

int main(int argc, char *argv[])
{
    int i, ret;

    if (argc < 2)
    {
        help_func(0, NULL);
        return 1;
    }

    xch = xc_interface_open(0, 0, 0);
    if ( !xch )
    {
        fprintf(stderr, "Failed to open an xc handler");
        return 1;
    }

    for ( i = 0; i < ARRAY_SIZE(opts); i++ )
    {
        if ( !strncmp(opts[i].name, argv[1], strlen(argv[1])) )
            break;
    }

    if ( i == ARRAY_SIZE(opts) )
    {
        fprintf(stderr, "Unknown option '%s'", argv[1]);
        help_func(0, NULL);
        return 1;
    }

    ret = opts[i].func(argc - 2, argv + 2);

    xc_interface_close(xch);

    return !!ret;
}

/*
 * Local variables:
 * mode: C
 * c-set-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
