#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>

#include "cfuc_driver.h"
#include "rust_additional.h"

int readCANFrameFromSocket(int socket,uint8_t* buff, struct timeval * tv)
{
	int nbytes;
    nbytes = read(socket, buff, CANFD_MTU);
    if (nbytes == CANFD_MTU) {
            printf("got CAN FD frame with length %d\n", ((struct canfd_frame*)buff)->len);
    } else if (nbytes == CAN_MTU) {
            printf("got legacy CAN frame with length %d\n", ((struct can_frame*)buff)->can_dlc);
    } else {
            // fprintf(stderr, "read: invalid CAN(FD) frame\n");
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
		// /* send frame */
	if (write(socket, frame, CANFD_MTU) != CANFD_MTU) {
		perror("write");
		return 1;
	}
}

int main(int argc, char **argv)
{
	static uint8_t can_buff[CANFD_MTU];
	static uint8_t can_buff_usb[CANFD_MTU];
		
	int s; /* can raw socket */ 
	struct sockaddr_can addr;
	int running = 1;
	struct can_filter fi;
	int enable_canfd = 1;

    struct timeval tv;

	/* check command line options */
	if (argc != 3) {
		fprintf(stderr, "%s: adapter for applications using"
			" the uCAN USB protocol.\n", basename(argv[0]));
		fprintf(stderr, "Usage: %s <usb id> <can interface>\n", basename(argv[0]));
		fprintf(stderr, "\nExamples:\n");
		fprintf(stderr, "%s can0 usb01  - creates can0 for tinterface for usb device\n\n",
			basename(argv[0]));
		fprintf(stderr, "\n");
		return 1;
	}

	/* open usblib uccb */
	if (cfuc_open_device() < 0) {
		printf("error openig USB device \n");
		goto usb_not_opened;
	};
	printf("USB device opened\n");
	
	

	/* open socket */
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = if_nametoindex(argv[2]);

	/* interface is ok - try to switch the socket into CAN FD mode */
	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
				&enable_canfd, sizeof(enable_canfd))){
		printf("error when enabling CAN FD support\n");
		return 1;
	}

	fcntl(s, F_SETFL, O_NONBLOCK);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* open filter by default */
	fi.can_id   = 0;
	fi.can_mask = 0;

	while (running) {

		uint8_t ftype = readCANFrameFromSocket(s,can_buff,&tv);
		if (ftype == CANFD_MTU)
		{
			cfuc_canfd_tx((struct canfd_frame*)can_buff,&tv);
		} else if (ftype == CAN_MTU)
		{
			cfuc_can_tx((struct can_frame*)can_buff,&tv);
		}
		
		if (cfuc_get_frame_from_usb(can_buff_usb) == 0)
		{// new data from usb
			writeCANFrameToSocket(s,can_buff_usb);
		}
	}

	cfuc_close_device();
	close(s);
usb_not_opened:
	
	return 0;
}
