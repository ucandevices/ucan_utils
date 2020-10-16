#ifndef CFUC_ARGS_H
#define CFUC_ARGS_H


typedef struct 
{
    int usb_serial;
    const char *can_interface_name;
    int gotoboot;
}t_cfuc_args;

t_cfuc_args* parse_args(int argc, char **argv);

#endif /*CFUC_ARGS_H*/