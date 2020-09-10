#include <stdio.h>
#include <libusb.h>
#include <signal.h>
#include "./stm32g4xx_hal_fdcan.h"
#include "./ucan_fd_protocol_stm32g431.h"

#define EP_CMD (1 | LIBUSB_ENDPOINT_OUT)

static struct libusb_device_handle *devh = NULL;


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


int uccb_open_device(void)
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

    bulkres = libusb_bulk_transfer(devh, EP_CMD, (unsigned char*)&ucan_initframe, sizeof(ucan_initframe), &tranfered, 100);
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

int uccb_close_device(void)
{
    libusb_release_interface(devh,0);
    libusb_close(devh);
    libusb_exit(NULL); 

    return 0;
}

