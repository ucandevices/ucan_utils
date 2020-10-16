#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include "argparse.h"
#include "cfuc_args.h"


t_cfuc_args cfuc_args = {
    0,
    NULL,
    0,
};


static const char *const usage[] = {
    "test_argparse [options] [[--] args]",
    "test_argparse [options]",
    NULL,
};

#define PERM_READ  (1<<0)
#define PERM_WRITE (1<<1)
#define PERM_EXEC  (1<<2)

t_cfuc_args* parse_args(int argc, char **argv) 
{
    int force = 0;
    int test = 0;
    int num = 0;
    const char *path = NULL;
    int perms = 0;
    struct argparse_option options[] = {
        OPT_HELP(),
        OPT_BOOLEAN('b', "boot", &cfuc_args.gotoboot, "go to bootlader mode"),
        OPT_STRING('c', "can_interface", &cfuc_args.can_interface_name, "CAN interface"),
        OPT_INTEGER('u', "usb_serial", &num, "USB CFUC SERIAL ID"),
        OPT_END(),
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usage, 0);
    argparse_describe(&argparse, "\nAdapter for applications using the uCAN USB protocol.", "\nExample: cfuc_adapter can0 21354  - attaches to can0 socketCAN interface usb device with serial 21354.");
    argc = argparse_parse(&argparse, argc, argv);
    return &cfuc_args;
}