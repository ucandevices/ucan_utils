#include <stdio.h>
#include <string.h>
#include <libusb.h>
#include <signal.h>
#include <linux/can.h>
#include <linux/can/raw.h>

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

void cfuc_print_status(UCAN_AckFrameDef* ackp)
{
            /* print device status can_protocol_status*/
        printf("CAN_S ");
        switch (ackp->can_protocol_status.Activity)
        {
            case FDCAN_COM_STATE_SYNC:
                printf("SYNC ");
            break;
            case FDCAN_COM_STATE_IDLE:
                printf("IDLE ");
            break;
            case FDCAN_COM_STATE_RX:
                printf("RX   ");
            break;
            case FDCAN_COM_STATE_TX:
                printf("TX   ");
            break;
        }
        if (ackp->can_protocol_status.ErrorPassive) printf ("ErrPassive "); else printf ("ErrActive  ");
        if (ackp->can_protocol_status.BusOff) printf ("BussOff "); else printf ("BussOn  ");        
        
        switch (ackp->can_protocol_status.LastErrorCode)
        {
            case FDCAN_PROTOCOL_ERROR_NONE:
                printf("IdErr:NONE ");    
                break;
            case FDCAN_PROTOCOL_ERROR_STUFF:
                printf("IdErr:STUF ");    
                break;    
            case FDCAN_PROTOCOL_ERROR_FORM:
                printf("IdErr:FORM ");    
                break;
            case FDCAN_PROTOCOL_ERROR_ACK:
                printf("IdErr:ACK  ");    
                break;
            case FDCAN_PROTOCOL_ERROR_BIT1:
                printf("IdErr:BIT1 ");    
                break;
            case FDCAN_PROTOCOL_ERROR_BIT0:
                printf("IdErr:BIT0 ");    
                break;
            case FDCAN_PROTOCOL_ERROR_CRC:
                printf("IdErr:CRC  ");    
                break;
            default:
                printf("IdErr:NOCHG ");    
                break;
        }
        // printf("DataErr:%1x ", ackp->can_protocol_status.DataLastErrorCode);
        /* print device status can_error_counters*/
        printf("TxE:%1x ", ackp->can_error_counters.TxErrorCnt);
        printf("RxE:%1x ", ackp->can_error_counters.RxErrorCnt);
        printf("LgE:%1x ", ackp->can_error_counters.ErrorLogging);
        printf("\r\n");
}

int cfuc_request_status(void)
{
    UCAN_Get_CAN_Status status = {UCAN_FD_GET_CAN_STATUS};
    if (cfuc_send_to_usb((uint8_t *)&status, sizeof(status)))
    {
        log_debug("Get status failed");
    }

    if (cfuc_get_ack((unsigned char *)&ucan_ackframe))
    {
        log_error("error ack init");
        return -1;
    } else
    {
       
    }
    return 0;
}
static unsigned char usb_serial[15];
static struct libusb_device_descriptor desc;

static libusb_hotplug_callback_handle cfuc_connect_handle;
static libusb_hotplug_callback_handle cfuc_disconnect_handle;

static CFUC_USB_STATUS cfuc_usb_status;

CFUC_USB_STATUS cfuc_is_connected()
{
    return cfuc_usb_status;
}

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

void cfuc_connect_handle_f(struct libusb_context *ctx, struct libusb_device *dev,
                     libusb_hotplug_event event, void *user_data)
{
    log_debug("-----------CFUC_CONNECT-----------");
    cfuc_open_device();
}
void cfuc_disconnect_handle_f(struct libusb_context *ctx, struct libusb_device *dev,
                     libusb_hotplug_event event, void *user_data)
{
    log_debug("-----------CFUC_DISCONNECT--------------");          
    cfuc_close_device();
}

libusb_device *dev;
libusb_context *ctx;

int cfuc_handle_usb_events(void)
{
    static struct timeval tv;
    tv.tv_usec = 100;
    libusb_handle_events_timeout(NULL,&tv);

    if (cfuc_usb_status == CFUC_USB_OPENED)
    {
        libusb_set_auto_detach_kernel_driver(devh, 1);

        if (libusb_claim_interface(devh, 1) < 0)
        {
            libusb_release_interface(devh,1);
            log_error("error claim if");
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
            } else {
                cfuc_usb_status = CFUC_CAN_OPENED;
            }
        }  
    }
    return 0;
}

int cfuc_init(FDCAN_InitTypeDef *init_data, unsigned char *serial)
{
    int rc;

    memcpy((void *)&(ucan_initframe.can_init), init_data, sizeof(FDCAN_InitTypeDef));

    log_debug("USB_INIT");

    libusb_init(&ctx);
    // if (libusb_init(&ctx) < 0)
    // {
    //     log_error("failed to initialise libusb");
    //     libusb_exit(ctx);
    //     return -1;
    // }
    cfuc_serial = serial;

    rc =  libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, 0, CFUC_VID, CFUC_PID, LIBUSB_HOTPLUG_MATCH_ANY, cfuc_connect_handle_f, NULL, &cfuc_connect_handle);

    if (LIBUSB_SUCCESS != rc) {
        log_debug("Error creating a hotplug callback\n");
    }

    rc = cfuc_disconnect_handle = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_NO_FLAGS, CFUC_VID, CFUC_PID, LIBUSB_HOTPLUG_MATCH_ANY, cfuc_disconnect_handle_f, NULL, &cfuc_disconnect_handle);

    if (LIBUSB_SUCCESS != rc) {
        log_debug("Error creating a hotplug callback\n");
    }
    return 0;
}

int cfuc_open_device(void)
{
    dev = cfuc_find_device(ctx, cfuc_serial);
    if (dev == NULL)
    {
        log_error("device not found NULL error");
        return -1;
    }

    int r = libusb_open(dev, &devh);
    printf("libusb_open %d\r\n",r);
    if (devh == NULL)
    {
        log_error("error open %i", r);
        return -1;
        
    }
    cfuc_usb_status = CFUC_USB_OPENED;
    return 0;
}

int cfuc_deinit(void)
{
  log_debug("DEINIT USB");

  libusb_hotplug_deregister_callback(NULL, cfuc_connect_handle);
  libusb_hotplug_deregister_callback(NULL, cfuc_disconnect_handle);
  libusb_exit(ctx);
}
int cfuc_close_device(void)
{
    cfuc_usb_status = CFUC_USB_INIT;
    if (devh != NULL)
    {
        // printf("e1\r\n");
        // libusb_release_interface(devh, 0);   
        log_debug("cfuc_close_device\r\n");
        libusb_close(devh);
        devh = NULL;
        // libusb_exit(NULL);
    }   
    return 0;
}

static UCAN_TxFrameDef cfuc_tx;
static UCAN_GoToBootladerFrameDef cfuc_boot;

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

int cfuc_can_tx(struct can_frame *frame, struct timeval *tv)
{
    tv = tv;
    cfuc_tx.frame_type = UCAN_FD_TX;
    cfuc_tx.can_tx_header.DataLength = ((uint32_t)(frame->can_dlc)) << 16;
    cfuc_tx.can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    cfuc_tx.can_tx_header.Identifier = frame->can_id;
    // cfuc_tx.can_tx_header.TxFrameType = FDCAN_REMOTE_FRAME
    memcpy((void *)cfuc_tx.can_data, (void *)frame->data, frame->can_dlc);
    
    if (cfuc_send_to_usb((uint8_t *)&cfuc_tx, sizeof(cfuc_tx)))
        return -1;
    return cfuc_get_ack((unsigned char *)&ucan_ackframe);
    
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

    if (cfuc_send_to_usb((uint8_t *)&cfuc_tx, sizeof(cfuc_tx)))
        return -1;
    return cfuc_get_ack((unsigned char *)&ucan_ackframe);
}

int cfuc_send_to_usb(uint8_t *usb_buff, int frame_size)
{
    int tranfered = 0;
    int tranfered_total = 0;
    int r, i;

    log_debug("USB %02X< ", frame_size);
    // int loop;
    // for (loop = 0; loop < frame_size; loop++)
    //     log_debug("%02X ", usb_buff[tranfered_total + loop]);
    // log_debug("\n");

    usb_counter++;
    do
    {
        if (devh != NULL)
        {
            // The transfer length must always be exactly 31 bytes.
            r = libusb_bulk_transfer(devh, EP_OUT, &(usb_buff[tranfered_total]), frame_size, &tranfered, 100);
            if (r == LIBUSB_ERROR_PIPE)
            {
                libusb_clear_halt(devh, EP_OUT);
            }
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
    
    cfuc_print_status(buff_frame);   
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
    if (devh != NULL)
    {
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
    }

    return -1;
}
