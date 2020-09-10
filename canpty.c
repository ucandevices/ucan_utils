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

// max frame size 
#define SLC_MTU 64u

int can2pty(int pty, int socket, struct canfd_frame *frame, struct timeval * tv)
{
	int nbytes;
	char cmd;
	char buf[SLC_MTU];
	int ptr;

	int i;

	nbytes = read(socket, &frame, sizeof(frame));
	if (nbytes != sizeof(frame)) {
		perror("read socket");
		return 1;
	}

    if (ioctl(socket, 0x8906, &tv) < 0)
    {
        perror("SIOCGSTAMP");
	}
	return 0;
}
