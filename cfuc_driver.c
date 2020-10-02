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
    {.ClockDivider = 0,
     .FrameFormat = FDCAN_FRAME_CLASSIC,
     .Mode = FDCAN_MODE_INTERNAL_LOOPBACK,
     .AutoRetransmission = DISABLE,
     .TransmitPause = DISABLE,
     .NominalPrescaler = 1,
     .NominalSyncJumpWidth = 1,
     .NominalTimeSeg1 = 2,
     .NominalTimeSeg2 = 2,
     .DataPrescaler = 1,
     .DataSyncJumpWidth = 1,
     .DataTimeSeg1 = 1,
     .DataTimeSeg2 = 1,
     .StdFiltersNbr = 0,
     .ExtFiltersNbr = 0,
     .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION}};

UCAN_AckFrameDef ucan_ackframe;

int cfuc_open_device(void)
{
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
         
        } else 
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
    cfuc_tx.can_tx_header.DataLength = (uint32_t)frame->can_dlc;
    cfuc_tx.can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    memcpy((void *)cfuc_tx.can_data, (void *)frame->data, cfuc_tx.can_tx_header.DataLength);

    return cfuc_send_to_usb((uint8_t *)&cfuc_tx, sizeof(cfuc_tx));
}

int cfuc_canfd_tx(struct canfd_frame *frame, struct timeval *tv)
{
    tv = tv;
    cfuc_tx.frame_type = UCAN_FD_TX;
    cfuc_tx.can_tx_header.DataLength = (uint32_t)frame->len << 16;
    cfuc_tx.can_tx_header.FDFormat = FDCAN_FD_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    memcpy((void *)cfuc_tx.can_data, (void *)frame->data, cfuc_tx.can_tx_header.DataLength);

    return cfuc_send_to_usb((uint8_t *)&cfuc_tx, sizeof(cfuc_tx));
}

int cfuc_send_to_usb(uint8_t *usb_buff, int frame_size)
{
    int tranfered = 0;
    const int MAX_CH_SIZE = 64;
    int tranfered_total = 0;
    int r, i;

    log_debug("USB %02X< ", frame_size);
    int loop;
    // for (loop = 0; loop < frame_size; loop++)
    //     log_debug("%02X ", usb_buff[tranfered_total + loop]);
    // log_debug("\n");

    usb_counter ++;
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
        log_debug("usb_counter %02X",usb_counter);
        log_debug("libusb_bulk_transfer failed: %s", libusb_error_name(r));
        cfuc_close_device();
        cfuc_open_device();
        return -1;
    }

    // /* one bulk tranfer in USB 2.0 can have 64 bytes maximum */
    // while (tranfered_total < frame_size)
    // {

    //     int left_to_tranfer = frame_size - tranfered_total;
    //     int chunk_size = frame_size;
    //     if (left_to_tranfer >= MAX_CH_SIZE)
    //     {
    //         chunk_size = MAX_CH_SIZE;
    //     }
    //     else
    //     {
    //         chunk_size = left_to_tranfer;
    //     }
    //     printf("chunk_size 0x%02X\n", chunk_size);
    //     // getchar();

    //     printf("USB %02X< ", chunk_size);
    //     int loop;
    //     for (loop = 0; loop < chunk_size; loop++)
    //         printf("%02X ", usb_buff[tranfered_total + loop]);
    //     printf("\n");

    //     // tranfered = chunk_size;

    //     do
    //     {
    //         // The transfer length must always be exactly 31 bytes.
    //         r = libusb_bulk_transfer(devh, EP_OUT, &(usb_buff[tranfered_total]), chunk_size, &tranfered, 100);
    //         if (r == LIBUSB_ERROR_PIPE)
    //         {
    //             libusb_clear_halt(devh, EP_OUT);
    //         }
    //         i++;
    //     } while ((r == LIBUSB_ERROR_PIPE) && (i < RETRY_MAX));
    //     if (r != LIBUSB_SUCCESS)
    //     {
    //         printf("libusb_bulk_transfer failed: %s\n", libusb_error_name(r));
    //         return -1;
    //     }

    //     tranfered_total += chunk_size;
    //     if (tranfered != chunk_size)
    //     {
    //         printf("USB ERR: Send %d expected %d\n", tranfered, chunk_size);
    //         return -1;
    //     }
    // }
    // return 0;
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
        log_debug("frame_type is %02X",ack->frame_type);
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
            uint8_t can_len = rx->can_rx_header.DataLength;            
            can->can_id = rx->can_rx_header.Identifier;
            can->can_dlc = can_len;
            log_debug("CAN> ID:%d L:%d",can->can_id,can->can_dlc);
            memcpy((void *)can->data, (void *)rx->can_data, can_len);
            return 0;
        }
        else // FDCAN
        {
            struct canfd_frame *fdcan = (struct canfd_frame *)buff_frame;
            uint8_t can_len = rx->can_rx_header.DataLength >> 16;
            fdcan->can_id = rx->can_rx_header.Identifier;
            fdcan->len = can_len;
            log_debug("CANFD> ID:%d L:%d",fdcan->can_id,fdcan->len);
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
    bulkres = libusb_bulk_transfer(devh, EP_IN, usb_buff, len, &tranfered, 100);
    if (bulkres == 0)
    {
        if (tranfered > 0)
        {
            log_debug("USB %02X> ", tranfered);
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
