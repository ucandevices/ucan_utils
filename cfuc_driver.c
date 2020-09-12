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

#define EP_OUT (1 | LIBUSB_ENDPOINT_OUT)
#define EP_IN (1 | LIBUSB_ENDPOINT_IN)

static struct libusb_device_handle *devh = NULL;
static int cfuc_send_to_usb(struct timeval * tv);

// init struct
UCAN_InitFrameDef ucan_initframe = {
    UCAN_FD_INIT,
    {
        .ClockDivider = 0,
        .FrameFormat =  FDCAN_FRAME_CLASSIC,
        .Mode = FDCAN_MODE_EXTERNAL_LOOPBACK,
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
        .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION
    }
};


int cfuc_open_device(void)
{
    int bulkres = 0;
    int tranfered = 0;
    if (libusb_init(NULL) < 0)
    {
        printf("failed to initialise libusb\n");
        return -1;
    }
    devh = libusb_open_device_with_vid_pid(NULL, 0x1209, 0x0775);

    if (devh == NULL) 
    {
        printf("error open\n");
        libusb_close(devh);
        libusb_exit(NULL);
        return -1;
    }
    if(libusb_claim_interface(devh, 1) < 0) 
    {
        printf("error claim if\n");        
        libusb_close(devh);
        libusb_exit(NULL);
        return -1;
    }

    bulkres = libusb_bulk_transfer(devh, EP_OUT, (unsigned char*)&ucan_initframe, sizeof(ucan_initframe), &tranfered, 100);
    if (tranfered != sizeof(ucan_initframe))
    {
        printf("Err INIT UCFD %i\n",tranfered);
        libusb_close(devh);
        libusb_exit(NULL);
        return -1;
    }
ucan_initframe:
   

    return 0;
}

int cfuc_close_device(void)
{
    libusb_release_interface(devh,0);
    libusb_close(devh);
    libusb_exit(NULL); 

    return 0;
}

static UCAN_TxFrameDef cfuc_tx;

int cfuc_can_tx(struct can_frame* frame, struct timeval * tv)
{
    cfuc_tx.frame_type = UCAN_FD_TX;
    cfuc_tx.can_tx_header.DataLength = (uint32_t)frame->can_dlc;
    cfuc_tx.can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    memcpy((void*)cfuc_tx.can_data,(void*)frame->data,cfuc_tx.can_tx_header.DataLength);

    return cfuc_send_to_usb(tv);
}

int cfuc_canfd_tx(struct canfd_frame* frame, struct timeval * tv)
{
    cfuc_tx.frame_type = UCAN_FD_TX;
    cfuc_tx.can_tx_header.DataLength = (uint32_t)frame->len << 16;
    cfuc_tx.can_tx_header.FDFormat = FDCAN_FD_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    memcpy((void*)cfuc_tx.can_data,(void*)frame->data,cfuc_tx.can_tx_header.DataLength);

    return cfuc_send_to_usb(tv);
}

static int cfuc_send_to_usb(struct timeval * tv)
{   
    int bulkres = 0;
    int tranfered = 0;

    printf("Send to USB\n");
    bulkres = libusb_bulk_transfer(devh, EP_OUT, (unsigned char*)&cfuc_tx, sizeof(ucan_initframe), &tranfered, 100);
    if (tranfered != sizeof(ucan_initframe))
    {
        printf("Err INIT UCFD %i\n",tranfered);
        libusb_close(devh);
        libusb_exit(NULL);
        return -1;
    }
    return 0;
}

int cfuc_get_frame_from_usb(uint8_t* buff_frame)
{
    int bulkres;
    int tranfered;
    static uint8_t usb_buff[MAX_CFUC_USB_FRAME_SIZE] = {0, 1, 2, 3, 4};
  
    bulkres = libusb_bulk_transfer(devh, EP_IN, usb_buff, MAX_CFUC_USB_FRAME_SIZE, &tranfered, 1);
    if(bulkres == 0)
    {
        printf("Transfer result %i \n", bulkres);
        if (tranfered > 0)
        { 
            printf("USB %d> ", tranfered);
            int loop;
            for(loop = 0; loop < tranfered; loop++)
                printf("%02X ", usb_buff[loop]);
            printf("\n");

            UCAN_RxFrameDef* rx = (UCAN_RxFrameDef*)usb_buff;
            if (rx->frame_type == UCAN_FD_RX)
            {
                if(rx->can_rx_header.FDFormat == FDCAN_CLASSIC_CAN)
                {
                    struct can_frame* can = (struct can_frame*)buff_frame;
                    uint8_t can_len = rx->can_rx_header.DataLength;
                    printf("CAN rx\n");
                    can->can_id = rx->can_rx_header.Identifier;
                    can->can_dlc = can_len;
                    memcpy((void*)can->data,(void*)rx->can_data,can_len);  

                    return 0;

                } else // FDCAN
                {
                    struct canfd_frame* fdcan = (struct canfd_frame*)buff_frame;
                    uint8_t can_len = rx->can_rx_header.DataLength >> 16;
                    printf("CANFD rx\n");
                    fdcan->can_id = rx->can_rx_header.Identifier;
                    fdcan->len = can_len;
                    memcpy((void*)fdcan->data,(void*)rx->can_data,can_len);  

                    return 0;   
                }                
            } 
        }
    }
    return -1;
}

