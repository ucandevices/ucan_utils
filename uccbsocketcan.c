#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#include "uccb.h"

extern int can2pty(int pty, int socket, struct canfd_frame *frame, struct timeval * tv);

int main(int argc, char **argv)
{
	fd_set rdfs;
	int p; /* pty master file */ 
	int s; /* can raw socket */ 
	struct sockaddr_can addr;
	int select_stdin = 0;
	int running = 1;
	int tstamp = 0;
	int is_open = 0;
	struct can_filter fi;
	int enable_canfd = 1;

	struct canfd_frame frame;
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
	// if (uccb_open_device() < 0) {
		// printf("error openig USB device \n");
		// goto usb_not_opened;
	// };

	printf("USB device opened\n");
	

	/* open socket */
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = if_nametoindex(argv[2]);

	// if (mtu != CANFD_MTU) {
			// printf("CAN interface is not CAN FD capable - sorry.\n");
			// return 1;
		// }

	/* interface is ok - try to switch the socket into CAN FD mode */
	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
				&enable_canfd, sizeof(enable_canfd))){
		printf("error when enabling CAN FD support\n");
		return 1;
	}

	/* disable reception of CAN frames until we are opened by 'O' */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	// /* send frame */
	// if (write(s, &frame, required_mtu) != required_mtu) {
	// 	perror("write");
	// 	return 1;
	// }

	/* open filter by default */
	fi.can_id   = 0;
	fi.can_mask = 0;

	while (running) {

		FD_ZERO(&rdfs);

		if (select_stdin)
			FD_SET(0, &rdfs);

		FD_SET(p, &rdfs);
		FD_SET(s, &rdfs);

		if (select(s+1, &rdfs, NULL, NULL, NULL) < 0) {
			perror("select");
			return 1;
		}

	if (FD_ISSET(s, &rdfs))
	{
		if (can2pty(p, s, &frame, &tv )) {
		
		}
	}
		


		if (FD_ISSET(0, &rdfs)) {
			running = 0;
			continue;
		}
		
	}

	uccb_close_device();
	close(p);
	close(s);
usb_not_opened:
	


	return 0;
}
