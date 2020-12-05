#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include "argparse.h"
#include "cfuc_args.h"


t_cfuc_args cfuc_args = {
    .usb_serial = 0,
    .can_interface_name = NULL,
    .gotoboot = 0,
    .id_baud = -1,
    .data_baud = -1,
    .is_fd = NULL,
    .mode = NULL,
    .verbose = 0
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
        OPT_BOOLEAN('\0', "boot", &cfuc_args.gotoboot, "go to bootlader mode"),
        OPT_STRING('c', "can_interface", &cfuc_args.can_interface_name, "CAN interface"),
        OPT_STRING('u', "usb_serial", &cfuc_args.usb_serial, "USB CFUC SERIAL ID in HEX"),
        OPT_INTEGER('d', "data_baud", &cfuc_args.data_baud, "CAN DATA BAUDRATE kBaud"),
        OPT_INTEGER('i', "id_baud", &cfuc_args.id_baud, "ID BAUDRATE in kBaud"),
        OPT_STRING('f', "frame_type", &cfuc_args.is_fd, "'c' for CLASSIC / b for BRS / n for noBRS"),
        OPT_STRING('m', "mode", &cfuc_args.mode, "'n' for NORMAL / 'm' for BUS_MONITORING / 'e' EXTERNAL_LOOPBACK / 'i' INTERNAL_LOOPBACK"),        
        OPT_BOOLEAN('\0', "verbose", &cfuc_args.verbose, "verbose, additional logs"),        
        OPT_END(),
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usage, 0);
    argparse_describe(&argparse, "\nAdapter for applications using the uCAN USB protocol.", "\nExample: cfuc_adapter can0 21354  - attaches to can0 socketCAN interface usb device with serial 21354.");
    argc = argparse_parse(&argparse, argc, argv);
    return &cfuc_args;
}