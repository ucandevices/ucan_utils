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

static struct libusb_device_handle *devh = NULL;
int cfuc_send_to_usb(uint8_t *usb_buff, int tranfered);
int cfuc_get_ack(uint8_t *buff_frame);
int cfuc_get_blocking_from_usb(uint8_t *usb_buff, int len);
int usb_counter;

// init struct
UCAN_InitFrameDef ucan_initframe = {
    UCAN_FD_INIT,
    {}
};

UCAN_AckFrameDef ucan_ackframe;

int cfuc_get_status(void)
{
    UCAN_Get_CAN_Status status = {UCAN_FD_GET_CAN_STATUS};
    cfuc_send_to_usb((uint8_t *)&status, sizeof(status));

    if (cfuc_get_ack((unsigned char *)&ucan_ackframe))
    {
        log_error("error ack init");
        return -1;
    }
    return 0;
}

int cfuc_open_device(FDCAN_InitTypeDef *init_data)
{
    memcpy((void*)&(ucan_initframe.can_init),init_data,sizeof(FDCAN_InitTypeDef));
    
    while (1)
    {
        if (libusb_init(NULL) < 0)
        {
            log_error("failed to initialise libusb");
            return -1;
        }
        devh = libusb_open_device_with_vid_pid(NULL, 0x1209, 0x0775);

        if (devh == NULL)
        {
            log_error("error open");
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
    ucan_initframe_err2:
        libusb_exit(NULL);
        usleep(10000);
    }

    return -1;
}

int cfuc_close_device(void)
{
    libusb_release_interface(devh, 0);
    libusb_close(devh);
    libusb_exit(NULL);

    return 0;
}

static UCAN_TxFrameDef cfuc_tx;

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
    const int MAX_CH_SIZE = 64;
    int tranfered_total = 0;
    int r, i;

    // log_debug("USB %02X< ", frame_size);
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
        cfuc_close_device();
        cfuc_open_device(&(config.fdcanInitType));
        return -1;
    }
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
            // log_debug("USB %02X> ", tranfered);
            int loop;
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
