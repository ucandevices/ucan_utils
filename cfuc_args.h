#ifndef CFUC_ARGS_H
#define CFUC_ARGS_H


typedef struct 
{
    const char *usb_serial;
    const char *can_interface_name;
    int gotoboot;
    int id_baud;
    int data_baud;
    const char *is_fd;
    const char *mode;
    int verbose;
}t_cfuc_args;

t_cfuc_args* parse_args(int argc, char **argv);

#endif /*CFUC_ARGS_H*/