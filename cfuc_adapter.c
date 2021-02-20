/**
 * Copyright (C) uCAN Devices 
 * Use of this source code is governed by a MIT-style license that can be found
 * in the LICENSE file.
 * 
**/

#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/netlink.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <linux/sockios.h>

#include "ucan_cfg.h"
#include "cfuc_driver.h"
#include "rust_additional.h"
#include "ucan_fd_protocol_stm32g431.h"
#include "log.h"
#include "cfuc_args.h"

	static int counter = 0;

int readCANFrameFromSocket(int socket, uint8_t *buff, struct timeval *tv)
{
	int nbytes;

	nbytes = read(socket, buff, CANFD_MTU);
	if (nbytes == CANFD_MTU)
	{
	    log_debug("SCAN> FD %d [%d]", ((struct canfd_frame *)buff)->can_id,((struct canfd_frame *)buff)->len);		
	}
	else if (nbytes == CAN_MTU)
	{
		log_debug("SCAN> %d [%d]", ((struct can_frame *)buff)->can_id, ((struct can_frame *)buff)->can_dlc);
	}
	else
	{
		// fprintf(stderr, "read: invalid CAN(FD) frame");
		return -1;
	}

	if (ioctl(socket, 0x8906, &tv) < 0)
	{
		return -1;
		perror("SIOCGSTAMP");
	}
	return nbytes;
}

int writeCANFrameToSocket(int socket, uint8_t *frame)
{	
	if (write(socket, frame, CANFD_MTU) != CANFD_MTU)
	{
		if (errno != ENOBUFS) {
			perror("write");
			return 1;
		}	
	}
	log_debug("SCAN< FD %d [%d]", ((struct canfd_frame *)frame)->can_id,((struct canfd_frame *)frame)->len);	
	return 0;
}

static inline time_t gettime()
{
	return clock();
}

uint32_t bitrate_nominal = 1000000;

struct can_bittiming
{
	__u32 bitrate;		/* Bit-rate in bits/second */
	__u32 sample_point; /* Sample point in one-tenth of a percent */
	__u32 tq;			/* Time quanta (TQ) in nanoseconds */
	__u32 prop_seg;		/* Propagation segment in TQs */
	__u32 phase_seg1;	/* Phase buffer segment 1 in TQs */
	__u32 phase_seg2;	/* Phase buffer segment 2 in TQs */
	__u32 sjw;			/* Synchronisation jump width in TQs */
	__u32 brp;			/* Bit-rate prescaler */
} bt;

void at_exit_handler(int sig)
{
	log_debug("exit");
	printf("exit\r\n");
	fflush(stdout);

	cfuc_close_device();
	cfuc_deinit();
	signal(sig, SIG_IGN);
	exit(0);
}

int main(int argc, char **argv)
{
	static uint8_t can_buff[MAX_CFUC_USB_FRAME_SIZE];
	static uint8_t can_buff_usb[MAX_CFUC_USB_FRAME_SIZE];

	int s; /* can raw socket */
	struct sockaddr_can addr;
	int running = 1;
	struct can_filter fi;
	int enable_canfd = 1;

	struct timeval tv;

	// putenv("LIBUSB_DEBUG=4");
	/* check command line options */

	configuration *cfg = load_cfg("cfuc_adapter.ini");

	t_cfuc_args *cfuc_args = parse_args(argc, argv);
	/* parse arguments */
	if ((cfuc_args->can_interface_name == NULL) & (cfuc_args->gotoboot == 0))
	{ // wrong params return -1
		printf("Wrong params type --h for help \r\n");
		return -1;
	}

	if (cfuc_args->verbose)
	{
		log_set_level(1);
	}
	else
	{
		log_set_level(4);
	}

	if (cfuc_args->id_baud != -1)
	{
		if (cfuc_cal_baudrate(cfuc_args->id_baud * 1000, &bt))
		{
			printf("%7d ***id bitrate not possible***\n", bitrate_nominal);
			return -1;
		}
		else
		{
			cfg->fdcanInitType.NominalTimeSeg1 = bt.phase_seg1 + bt.prop_seg;
			cfg->fdcanInitType.NominalTimeSeg2 = bt.phase_seg2;
			cfg->fdcanInitType.NominalPrescaler = bt.brp;
			cfg->fdcanInitType.NominalSyncJumpWidth = bt.sjw;
		}
	}

	if (cfuc_args->data_baud != -1)
	{
		if (cfuc_cal_baudrate(cfuc_args->data_baud * 1000, &bt))
		{
			printf("%7d ***data bitrate not possible***\n", bitrate_nominal);
			return -1;
		}
		else
		{
			cfg->fdcanInitType.DataTimeSeg1 = bt.phase_seg1 + bt.prop_seg;
			cfg->fdcanInitType.DataTimeSeg2 = bt.phase_seg2;
			cfg->fdcanInitType.DataPrescaler = bt.brp;
			cfg->fdcanInitType.DataSyncJumpWidth = bt.sjw;
		}
	}

	if (cfuc_args->is_fd != NULL)
	{
		switch (cfuc_args->is_fd[0])
		{
		case 'c':
			cfg->fdcanInitType.FrameFormat = FDCAN_FRAME_CLASSIC;
			break;
		case 'b':
			cfg->fdcanInitType.FrameFormat = FDCAN_FRAME_FD_BRS;
			break;
		case 'n':
			cfg->fdcanInitType.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
			break;
		default:
			printf("FrameFormat argument incorrect %s\r\n", cfuc_args->is_fd);
			return -1;
			break;
		}
	}

	if (cfuc_args->mode != NULL)
	{
		switch (cfuc_args->mode[0])
		{
		case 'n':
			cfg->fdcanInitType.Mode = FDCAN_MODE_NORMAL;
			break;
		case 'm':
			cfg->fdcanInitType.Mode = FDCAN_MODE_BUS_MONITORING;
			break;
		case 'e':
			cfg->fdcanInitType.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
			break;
		case 'i':
			cfg->fdcanInitType.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
			break;
		default:
			printf("CANMODE argument incorrect");
			return -1;
			break;
		}
	}

	/* open usblib uccb */
	if (cfuc_init(&(cfg->fdcanInitType), cfuc_args->usb_serial))
	{
		log_error("error init USBLib");
		goto usb_not_opened;
	};

	/* check if device is allready attached */
	if (cfuc_open_device()  != 0)
	{
		log_error("Wating for device attachment");
	}

	/* add exit signal  */
	struct sigaction sa;
    sa.sa_handler = at_exit_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; /* Restart functions if
                                 interrupted by handler */
    if (sigaction(SIGINT, &sa, NULL) == -1){
        /* Handle error */;
	}

	/* check for bootlader flag */
	if (cfuc_args->gotoboot)
	{
		int cnt = 0;
		while (cfuc_is_connected() == 0) 
		{
			cnt ++; sleep(1); 
			if (cnt > 3) at_exit_handler(SIG_IGN);
		}
		log_info("GOTOBOOT");
		cfuc_canfd_goto_boot();
		return 0;
	}

	/* open socket */
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s < 0)
	{
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = if_nametoindex(cfuc_args->can_interface_name);

	/* interface is ok - try to switch the socket into CAN FD mode */
	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
				   &enable_canfd, sizeof(enable_canfd)))
	{
		log_error("error when enabling CAN FD support");
		return 1;
	}

	fcntl(s, F_SETFL, O_NONBLOCK);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("bind");
		return 1;
	}

	/* open filter by default */
	fi.can_id = 0;
	fi.can_mask = 0;

	time_t timestamp = gettime();

	while (running)
	{
		if (cfuc_is_connected()) /* only in case USB is connected */
		{
			uint8_t ftype = readCANFrameFromSocket(s, can_buff, &tv);
			if (ftype == CANFD_MTU)
			{
				timestamp = gettime();
				cfuc_canfd_tx((struct canfd_frame *)can_buff, &tv);
			}
			else if (ftype == CAN_MTU)
			{
				timestamp = gettime();
				cfuc_can_tx((struct can_frame *)can_buff, &tv);
			}
			if (cfuc_get_frame_from_usb(can_buff_usb) == 0)
			{ // new data from usb
				timestamp = clock();
				writeCANFrameToSocket(s, can_buff_usb);
			}
			if (((gettime() - timestamp)) > (9000))
			{
				timestamp = gettime();
				// no frame was recived/send to USB for long time check if usb connection pressent
				cfuc_request_status();
			}
		}
		// usleep(100);
		cfuc_handle_usb_events();
	}

	cfuc_close_device();
	close(s);
usb_not_opened:

	return 0;
}
