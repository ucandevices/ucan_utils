#ifndef CFUC_DRIVER_H
#define CFUC_DRIVER_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include "rust_additional.h"

#define MAX_CFUC_USB_FRAME_SIZE (256U)

int cfuc_open_device(void);
int cfuc_close_device(void);
int cfuc_get_frame_from_usb(uint8_t* buff_frame);

int cfuc_can_tx(struct can_frame* frame, struct timeval * tv);
int cfuc_canfd_tx(struct canfd_frame* frame, struct timeval * tv);

#endif /* CFUC_DRIVER_H */