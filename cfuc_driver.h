#ifndef CFUC_DRIVER_H
#define CFUC_DRIVER_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>

int cfuc_open_device(void);
int cfuc_close_device(void);
int cfuc_send_to_usb(struct canfd_frame *frame, struct timeval * tv);

#endif /* CFUC_DRIVER_H */