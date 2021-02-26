#ifndef CFUC_DRIVER_H
#define CFUC_DRIVER_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include "rust_additional.h"

#define MAX_CFUC_USB_FRAME_SIZE (1000U)

typedef enum {
    CFUC_USB_INIT,
    CFUC_USB_OPENED,
    CFUC_CAN_OPENED,
} CFUC_USB_STATUS;

int cfuc_open_device(void);
int cfuc_init(FDCAN_InitTypeDef *init_data, unsigned char* serial);
int cfuc_deinit(void);

int cfuc_handle_usb_events(void);

CFUC_USB_STATUS cfuc_is_connected();
int cfuc_close_device(void);

int cfuc_get_frame_from_usb(struct can_frame *buff_frame_can, int *no_can, struct canfd_frame *buff_frame_fdcan, int *no_fd);

int cfuc_can_tx(struct can_frame* frame, struct timeval * tv);
int cfuc_canfd_tx(struct canfd_frame* frame, struct timeval * tv);

int cfuc_request_status(void);
int cfuc_canfd_goto_boot(void);
int cfuc_send_to_usb(uint8_t *usb_buff, int tranfered);

#endif /* CFUC_DRIVER_H */