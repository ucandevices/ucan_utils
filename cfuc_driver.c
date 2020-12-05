#include <stdio.h>
#include <string.h>
#include <libusb.h>
#include <signal.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>

#include "./stm32g4xx_hal_fdcan.h"
#include "./ucan_fd_protocol_stm32g431.h"
#include "./cfuc_driver.h"
#include "./log.h"
#include "./ucan_cfg.h"

#define EP_OUT (1 | LIBUSB_ENDPOINT_OUT)
#define EP_IN (1 | LIBUSB_ENDPOINT_IN)
#define RETRY_MAX 5

#define CFUC_VID 0x1209
#define CFUC_PID 0x0775

static struct libusb_device_handle *devh = NULL;
unsigned char *cfuc_serial = NULL;
int cfuc_send_to_usb(uint8_t *usb_buff, int tranfered);
int cfuc_get_ack(uint8_t *buff_frame);
int cfuc_get_blocking_from_usb(uint8_t *usb_buff, int len);
int usb_counter;

// init struct
UCAN_InitFrameDef ucan_initframe = {
    UCAN_FD_INIT,
    {}};

UCAN_AckFrameDef ucan_ackframe;

int cfuc_get_status(void)
{
    UCAN_Get_CAN_Status status = {UCAN_FD_GET_CAN_STATUS};
    if (cfuc_send_to_usb((uint8_t *)&status, sizeof(status)))
    {
        log_debug("Get status reinit");
        cfuc_close_device(0);
        cfuc_open_device();
    }

    if (cfuc_get_ack((unsigned char *)&ucan_ackframe))
    {
        log_error("error ack init");
        return -1;
    }
    return 0;
}
static unsigned char usb_serial[15];
static struct libusb_device_descriptor desc;
libusb_device *cfuc_find_device(libusb_context *ctx, unsigned char *serial)
{
    libusb_device *ret_dev = 0;
    libusb_device **list;
    ssize_t num_devs, i;
    log_debug("cfuc_find_device");
    num_devs = libusb_get_device_list(ctx, &list);
    for (i = 0; i < num_devs; ++i)
    {
        libusb_device *dev = list[i];
        libusb_get_device_descriptor(dev, &desc);
        if ((desc.idVendor == CFUC_VID) && (desc.idProduct == CFUC_PID))
        {
            if (serial != 0)
            {
                static struct libusb_device_handle *h;
                libusb_open(dev, &h);
                long l = libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, usb_serial, sizeof(usb_serial));
                if (l != LIBUSB_SUCCESS)
                {
                    log_error("USB Cannot get descriptor err no %i", l);
                }
                libusb_close(h);

                if (strcmp(usb_serial, serial) == 0)
                {
                    log_debug("Serial Match");
                    ret_dev = dev;
                }
                else
                {
                    log_error("CFUC Serial is %s expected %s", usb_serial, serial);
                }
            }
            else
            {
                ret_dev = dev;
            }
        }
    }
    libusb_free_device_list(list, 1);
    return ret_dev;
}

void cfuc_connect_handle()
{
    log_debug("CFUC_CONNECT");
}
void cfuc_disconnect_handle()
{
    log_debug("CFUC_DISCONNECT");
}

libusb_device *dev;
libusb_context *ctx;

int cfuc_init(FDCAN_InitTypeDef *init_data, unsigned char *serial)
{
    memcpy((void *)&(ucan_initframe.can_init), init_data, sizeof(FDCAN_InitTypeDef));

    log_debug("can_init frame_format %d\n", ucan_initframe.can_init.FrameFormat);

    if (libusb_init(&ctx) < 0)
    {
        log_error("failed to initialise libusb");
        libusb_exit(ctx);
        return -1;
    }
    cfuc_serial = serial;

    libusb_hotplug_register_callback(ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_NO_FLAGS, CFUC_VID, CFUC_PID, LIBUSB_HOTPLUG_MATCH_ANY, cfuc_connect_handle, NULL, NULL);
    libusb_hotplug_register_callback(ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_NO_FLAGS, CFUC_VID, CFUC_PID, LIBUSB_HOTPLUG_MATCH_ANY, cfuc_disconnect_handle, NULL, NULL);
}

int cfuc_open_device(void)
{
shame_start:
    dev = cfuc_find_device(ctx, cfuc_serial);
    if (dev == NULL)
    {
        log_error("device not found NULL error");
        return -1;
    }

    while (1)
    {
        int r = libusb_open(dev, &devh);

        if (devh == NULL)
        {
            log_error("error open %i", r);
            goto ucan_initframe_err2;
        }

        libusb_set_auto_detach_kernel_driver(devh, 1);

        if (libusb_claim_interface(devh, 1) < 0)
        {
            log_error("error claim if");
            goto ucan_initframe_err;
        }

        log_debug("INIT CAN");
        if (cfuc_send_to_usb((unsigned char *)&ucan_initframe, sizeof(ucan_initframe)))
        {
            log_error("error tx init");
            goto ucan_initframe_err;
        }
        else
        {
            log_debug("INIT ACK");
            if (cfuc_get_ack((unsigned char *)&ucan_ackframe))
            {
                log_error("error ack init");
                goto ucan_initframe_err;
            }
            usb_counter = 0;
        }
        return 0;
    ucan_initframe_err:
        libusb_close(devh);
        libusb_exit(ctx);
        log_debug("libusb reinit");
        if (libusb_init(&ctx) < 0)
        {
            log_error("failed to initialise libusb");
            libusb_exit(ctx);
            return -1;
        }
        else
        {
            usleep(10000);
            goto shame_start;
        }

    ucan_initframe_err2:
        usleep(10000);
    }

    return -1;
}

int cfuc_close_device(int force)
{
    if (devh != NULL)
    {
        libusb_release_interface(devh, 0);
        if (force)
            libusb_close(devh);
    }
    if (force)
        libusb_exit(NULL);

    return 0;
}

static UCAN_TxFrameDef cfuc_tx;
static UCAN_GoToBootladerFrameDef cfuc_boot;

int cfuc_can_tx(struct can_frame *frame, struct timeval *tv)
{
    tv = tv;
    cfuc_tx.frame_type = UCAN_FD_TX;
    cfuc_tx.can_tx_header.DataLength = ((uint32_t)(frame->can_dlc)) << 16;
    cfuc_tx.can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    // cfuc_tx.can_tx_header.TxFrameType = FDCAN_REMOTE_FRAME
    memcpy((void *)cfuc_tx.can_data, (void *)frame->data, frame->can_dlc);

    return cfuc_send_to_usb((uint8_t *)&cfuc_tx, sizeof(cfuc_tx));
}

int cfuc_canfd_goto_boot(void)
{
    cfuc_boot.frame_type = UCAN_FD_GO_TO_BOOTLOADER;

    cfuc_send_to_usb((uint8_t *)&cfuc_boot, sizeof(cfuc_boot));
    if (cfuc_get_ack((unsigned char *)&ucan_ackframe))
    {
        log_error("error ack init");
        return -1;
    }
    return 0;
}

int cfuc_canfd_tx(struct canfd_frame *frame, struct timeval *tv)
{
    tv = tv;
    cfuc_tx.frame_type = UCAN_FD_TX;

    if (frame->len <= 8)
        cfuc_tx.can_tx_header.DataLength = (uint32_t)frame->len << 16;
    else
    {
        switch (frame->len)
        {
        case 12:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
            break;

        case 16:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_16;
            break;

        case 20:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
            break;

        case 24:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_24;
            break;

        case 32:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_32;
            break;

        case 48:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_48;
            break;

        case 64:
            cfuc_tx.can_tx_header.DataLength = FDCAN_DLC_BYTES_64;
            break;
        default:
            log_debug("ERR! CANFD< ID:%04X L:%02X ERR!", frame->can_id, frame->len);
            return -1;
        }
    }

    cfuc_tx.can_tx_header.FDFormat = FDCAN_FD_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    memcpy((void *)cfuc_tx.can_data, (void *)frame->data, frame->len);

    log_debug("CANFD< ID:%04X L:%02X", frame->can_id, frame->len);

    return cfuc_send_to_usb((uint8_t *)&cfuc_tx, sizeof(cfuc_tx));
}

int cfuc_send_to_usb(uint8_t *usb_buff, int frame_size)
{
    int tranfered = 0;
    int tranfered_total = 0;
    int r, i;

    log_debug("USB %02X< ", frame_size);
    int loop;
    // for (loop = 0; loop < frame_size; loop++)
    //     log_debug("%02X ", usb_buff[tranfered_total + loop]);
    // log_debug("\n");

    usb_counter++;
    do
    {
        // The transfer length must always be exactly 31 bytes.
        r = libusb_bulk_transfer(devh, EP_OUT, &(usb_buff[tranfered_total]), frame_size, &tranfered, 100);
        if (r == LIBUSB_ERROR_PIPE)
        {
            libusb_clear_halt(devh, EP_OUT);
        }
        i++;
    } while ((r == LIBUSB_ERROR_PIPE) && (i < RETRY_MAX));
    if (r != LIBUSB_SUCCESS)
    {
        log_debug("usb_counter %02X", usb_counter);
        log_debug("libusb_bulk_transfer failed: %s", libusb_error_name(r));
        return -1;
    }
    return 0;
}

int cfuc_get_ack(uint8_t *buff_frame)
{
    if (cfuc_get_blocking_from_usb(buff_frame, sizeof(UCAN_AckFrameDef)))
    {
        log_debug("rx bulk failed");
        return -1;
    }
    UCAN_AckFrameDef *ack = (UCAN_AckFrameDef *)buff_frame;
    if (ack->frame_type != UCAN_FD_ACK)
    {
        log_debug("frame_type is %02X", ack->frame_type);
        return -1;
    }
    return 0;
}

int cfuc_get_frame_from_usb(uint8_t *buff_frame)
{
    static uint8_t usb_buff[MAX_CFUC_USB_FRAME_SIZE] = {0, 1, 2, 3, 4};

    if (cfuc_get_blocking_from_usb(usb_buff, MAX_CFUC_USB_FRAME_SIZE))
    {
        return -1;
    }
    UCAN_RxFrameDef *rx = (UCAN_RxFrameDef *)usb_buff;
    if (rx->frame_type == UCAN_FD_RX)
    {
        if (rx->can_rx_header.FDFormat == FDCAN_CLASSIC_CAN)
        {
            struct can_frame *can = (struct can_frame *)buff_frame;

            uint32_t can_len = rx->can_rx_header.DataLength >> 16;
            can->can_id = rx->can_rx_header.Identifier;
            can->can_dlc = can_len;
            memcpy((void *)can->data, (void *)rx->can_data, can_len);
            log_debug("CAN> ID:%04X L:%02X D:%02X %02X %02X %02X %02X", can->can_id, can->can_dlc, can->data[0], can->data[1], can->data[2], can->data[3], can->data[4]);
            return 0;
        }
        else // FDCAN
        {
            struct canfd_frame *fdcan = (struct canfd_frame *)buff_frame;
            uint8_t can_len = rx->can_rx_header.DataLength >> 16;

            switch (rx->can_rx_header.DataLength)
            {
            case FDCAN_DLC_BYTES_12:
                can_len = 12;
                break;

            case FDCAN_DLC_BYTES_16:
                can_len = 16;
                break;

            case FDCAN_DLC_BYTES_20:
                can_len = 20;
                break;

            case FDCAN_DLC_BYTES_24:
                can_len = 24;
                break;

            case FDCAN_DLC_BYTES_32:
                can_len = 32;
                break;

            case FDCAN_DLC_BYTES_48:
                can_len = 48;
                break;

            case FDCAN_DLC_BYTES_64:
                can_len = 64;
                break;
                // default:
                // log_debug("ERR! CANFD> LEN ERR!");
                // return -1;
            }

            fdcan->can_id = rx->can_rx_header.Identifier;
            fdcan->len = can_len;
            log_debug("CANFD> ID:%04X L:%02X", fdcan->can_id, fdcan->len);
            memcpy((void *)fdcan->data, (void *)rx->can_data, can_len);
            return 0;
        }
    }
    return -1;
}

int cfuc_get_blocking_from_usb(uint8_t *usb_buff, int len)
{
    int bulkres;
    int tranfered;

    // log_debug("GET USB %i ", len);
    bulkres = libusb_bulk_transfer(devh, EP_IN, usb_buff, len, &tranfered, 10);
    if (bulkres == 0)
    {
        if (tranfered > 0)
        {
            log_debug("USB %02X> ", tranfered);
            // int loop;
            // for (loop = 0; loop < tranfered; loop++)
            //     log_debug("%02X ", usb_buff[loop]);
            // log_debug("\n");
            return 0;
        }
    }
    else
    {
        // log_debug("libusb_bulk_transfer failed: %s", libusb_error_name(bulkres));
    }

    return -1;
}
