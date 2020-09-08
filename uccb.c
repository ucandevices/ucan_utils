#include <stdio.h>
#include <libusb.h>
#include <signal.h>

#define EP_DATA (1 | LIBUSB_ENDPOINT_IN)

static struct libusb_device_handle *devh = NULL;

int uccb_open_device(void)
{
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

    //if (libusb_bulk_transfer())

    return 0;
}

int uccb_close_device(void)
{
    libusb_release_interface(devh,0);
    libusb_close(devh);
    libusb_exit(NULL); 

    return 0;
}

