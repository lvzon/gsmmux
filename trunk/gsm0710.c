/*
*
* GSM 07.10 Implementation with User Space Serial Ports
*
* Copyright (C) 2003  Tuukka Karvonen <tkarvone@iki.fi>
*
* Version 1.0 October 2003
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
* Usage:
* gsm0710 <device> [numberOfPorts] [first_ttyU_portnumber] [max_frame_size]
*
* Modified November 2004 by David Jander <david@protonic.nl>
*  - Hacked to use Pseudo-TTY's instead of the (obsolete?) USSP driver.
*  - Fixed some bugs which prevented it from working with Sony-Ericsson modems
*  - Seriously broke hardware handshaking.
*  - Changed commandline interface to use getopts:
*
* New Usage:
* gsm0710 [options] <pty1> <pty2> ...
*
* Options are:
*  -p <serport>    : Serial port device to connect to [/dev/modem]
*  -f <framsize>   : Maximum frame size [32]
*  -h  		   : Show this help message
*  <included by Cargnini>
*  -d              : debug mode (don't fork)
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <string.h>
#include <paths.h>
#include <sys/types.h>
#include <sys/stat.h>
//syslog
#include <syslog.h>

#include "buffer.h"
#include "gsm0710.h"

#define DEFAULT_NUMBER_OF_PORTS 3
#define WRITE_RETRIES 5
#define MAX_CHANNELS   32
//vitorio, only to use if necessary (don't ask in what i was thinking  when i wrote this)
#define TRUE	1
#define FALSE	0

#define UNKNOW_MODEM	0
#define MC35		1
#define GENERIC		2


static volatile int terminate = 0;
static int terminateCount = 0;

static int *ussp_fd;
static int serial_fd;
static Channel_Status *cstatus;
static int max_frame_size = 31; // The limit of Sony-Ericsson GM47

static GSM0710_Buffer *in_buf;  // input buffer

static int _debug = 0;
static pid_t the_pid;
int _priority;
int _modem_type;



#if 0
/* Opens USSP port for use.
*
* PARAMS:
* port - port number
* RETURNS
* file descriptor or -1 on error
*/
int ussp_open(int port)
{
	int fd;
	char name[] = "ser0\0";

	name[3] = (char) (0x30 + port);
	PDEBUG("Open serial port %s ", name);
	fd = open(name, O_RDWR | O_NONBLOCK);
	PDEBUG("done.\n");

	return fd;
}
#endif

/**
 * Returns success, when an ussp is opened.
 */
int ussp_connected(int port)
{
#if 0
	struct ussp_operation op;

	op.op = USSP_OPEN_RESULT;
	if (cstatus[port + 1].opened)
		op.arg = 0;
	else
		op.arg = -1;
	op.len = 0;
	write(ussp_fd[port], &op, sizeof(op));

	PDEBUG("USSP port %d opened.\n", port);
	return 0;
#else
	return 0;
#endif
}

/** Writes a frame to a logical channel. C/R bit is set to 1.
* Doesn't support FCS counting for UI frames.
*
* PARAMS:
* channel - channel number (0 = control)

* input   - the data to be written
* count   - the length of the data
* type    - the type of the frame (with possible P/F-bit)
*
* RETURNS:
* number of characters written
*/
int write_frame(int channel, const char *input, int count, unsigned char type)
{
	// flag, EA=1 C channel, frame type, length 1-2
	unsigned char prefix[5] = { F_FLAG, EA | CR, 0, 0, 0 };
	unsigned char postfix[2] = { 0xFF, F_FLAG };
	int prefix_length = 4, c;

	if(_debug)
		syslog(LOG_DEBUG,"send frame to ch: %d \n", channel);
	// EA=1, Command, let's add address
	prefix[1] = prefix[1] | ((63 & (unsigned char) channel) << 2);
	// let's set control field
	prefix[2] = type;

	// let's not use too big frames
	count = min(max_frame_size, count);

	// length
	if (count > 127)
	{
		prefix_length = 5;
		prefix[3] = ((127 & count) << 1);
		prefix[4] = (32640 & count) >> 7;
	}
	else
	{
		prefix[3] = 1 | (count << 1);
	}
	// CRC checksum
	postfix[0] = make_fcs(prefix + 1, prefix_length - 1);

	c = write(serial_fd, prefix, prefix_length);
	if (c != prefix_length)
	{
		if(_debug)
			syslog(LOG_DEBUG,"Couldn't write the whole prefix to the serial port for the virtual port %d. Wrote only %d  bytes.", channel, c);
		return 0;
	}
	if (count > 0)
	{
		c = write(serial_fd, input, count);
		if (count != c)
		{
			if(_debug)
				syslog(LOG_DEBUG,"Couldn't write all data to the serial port from the virtual port %d. Wrote only %d bytes.\n", channel, c);
			return 0;
		}
	}
	c = write(serial_fd, postfix, 2);
	if (c != 2)
	{
		if(_debug)
			syslog(LOG_DEBUG,"Couldn't write the whole postfix to the serial port for the virtual port %d. Wrote only %d bytes.", channel, c);
		return 0;
	}

	return count;
}

/* Handles received data from ussp device.
*
* This function is derived from a similar function in RFCOMM Implementation
* with USSPs made by Marcel Holtmann.
*
* PARAMS:
* buf   - buffer, which contains received data
* len   - the length of the buffer
* port  - the number of ussp device (logical channel), where data was
*         received
* RETURNS:
* the number of remaining bytes in partial packet
*/
int ussp_recv_data(unsigned char *buf, int len, int port)
{
#if 0
	int n, written;
	unsigned char pkt_buf[4096];
	struct ussp_operation *op = (struct ussp_operation *) pkt_buf, *top;
	struct termios *tiosp;
	int i;                      // size
	unsigned char msc[5] = { CR | C_MSC, 0x5, 0, 0, 1 };

	PDEBUG( "(DEBUG) %s chamada\n", __FUNCTION__);

	memcpy(pkt_buf, buf, len);
	n = len;
	op = (struct ussp_operation *) pkt_buf;

	for (top = op;
		/* check for partial packet - first, make sure top->len is actually in pkt_buf */
		((char *) top + sizeof(struct ussp_operation) <= ((char *) op) + n)
		&& ((char *) top + sizeof(struct ussp_operation) + top->len <= ((char *) op) + n);
		top = (struct ussp_operation *) (((char *) top) + top->len + sizeof(struct ussp_operation)))
	{

		switch (top->op)
		{
			case USSP_OPEN:
				ussp_connected(port);
				break;
			case USSP_CLOSE:
				PDEBUG("Close ussp port %d\n", port);
				break;
			case USSP_WRITE:
				written = 0;
				i = 0;
				// try to write 5 times
				while ((written += write_frame(port + 1, top->data + written,
											top->len - written, UIH)) != top->len && i < WRITE_RETRIES)
				{
					i++;
				}
				if (i == WRITE_RETRIES)
				{
					PDEBUG("Couldn't write data to channel %d. Wrote only %d bytes, when should have written %ld.\n",
						(port + 1), written, top->len);
				}
				break;
			case USSP_SET_TERMIOS:
				tiosp = (struct termios *) (top + 1);
				if ((tiosp->c_cflag & CBAUD) == B0 && (cstatus[(port + 1)].v24_signals & S_RTC) > 0)
				{
					// drop DTR
					PDEBUG("Drop DTR.\n");
					msc[2] = 3 | ((63 & (port + 1)) << 2);
					msc[3] = cstatus[(port + 1)].v24_signals & ~S_RTC;
					cstatus[(port + 1)].v24_signals = msc[3];
					write_frame(0, msc, 4, UIH);
				}
				else if ((tiosp->c_cflag & CBAUD) != B0 && (cstatus[(port + 1)].v24_signals & S_RTC) == 0)
				{
					// DTR up
					PDEBUG("DTR up.\n");
					msc[2] |= ((63 & (port + 1)) << 2);
					msc[3] = cstatus[(port + 1)].v24_signals | S_RTC;
					cstatus[(port + 1)].v24_signals = msc[3];
					write_frame(0, msc, 4, UIH);
				}
#ifdef DEBUG
				PDEBUG("Set termios for ussp port %d\n", port);
				PDEBUG("\tinput mode flags:   0x%04x\n", tiosp->c_iflag);
				PDEBUG("\toutput mode flags:  0x%04x\n", tiosp->c_oflag);
				PDEBUG("\tcontrol mode flags: 0x%04x\n", tiosp->c_cflag);
				PDEBUG("\tlocal mode flags:   0x%04x\n", tiosp->c_lflag);
				PDEBUG("\tline discipline:    0x%02x\n", tiosp->c_line);
				PDEBUG("\tcontrol characters: ");
				for (i = 0; i < NCCS; i++)
					PDEBUG("0x%02x ", tiosp->c_cc[i]);
				PDEBUG("\n");
				PDEBUG("\tinput speed:        0x%02x (%i)\n", tiosp->c_ispeed, tiosp->c_ispeed);
				PDEBUG("\toutput speed:       0x%02x (%i)\n", tiosp->c_ospeed, tiosp->c_ospeed);
#endif
				break;
			case USSP_MSC:
				PDEBUG("Modem signal change\n");
				msc[2] = 3 | ((63 & (port + 1)) << 2);
				msc[3] = S_DV;
				if ((top->arg & USSP_DTR) == USSP_DTR)
				{
					msc[3] |= S_RTC;
					PDEBUG("RTC\n");
				}
				if ((top->arg & USSP_RTS) == USSP_RTS)
				{
					msc[3] |= S_RTR;
					PDEBUG("RTR\n");
				}
				else
				{
					msc[3] |= S_FC;
					PDEBUG("FC\n");
				}
				cstatus[(port + 1)].v24_signals = msc[3];       // save the signals
				write_frame(0, msc, 4, UIH);
				break;
			default:
				PDEBUG("Unknown code: %d\n", top->op);
				break;
		}

	}

	/* remaining bytes in partial packet */
	return ((char *) op + n) - (char *) top;
#else
	int written = 0;
	int i = 0;
	// try to write 5 times
	while ((written += write_frame(port + 1, buf + written,
								len - written, UIH)) != len && i < WRITE_RETRIES)
	{
		i++;
	}
	if (i == WRITE_RETRIES)
	{
		if(_debug)
			syslog(LOG_DEBUG,"Couldn't write data to channel %d. Wrote only %d bytes, when should have written %ld.\n",
			(port + 1), written, (long)len);
	}
	return 0;
#endif
}


int ussp_send_data(unsigned char *buf, int n, int port)
{
#if 0
	struct ussp_operation *op;

	op = malloc(sizeof(struct ussp_operation) + n);

	op->op = USSP_READ;
	op->arg = 0;
	op->len = n;
	memcpy(op->data, buf, n);

	write(ussp_fd[port], op, sizeof(struct ussp_operation) + n);

	free(op);
#else
	if(_debug)
		syslog(LOG_DEBUG,"send data to port ptya%d\n", port);
	write(ussp_fd[port], buf, n);
#endif
	return n;
}

/* Sends an AT-command to a given serial port and waits
* for reply.
*
* PARAMS:
* fd  - file descriptor
* cmd - command
* to  - how many microseconds to wait for response (this is done 100 times)
* RETURNS:
* 1 on success (OK-response), 0 otherwise
*/
int at_command(int fd, char *cmd, int to)
{
	fd_set rfds;
	struct timeval timeout;
	unsigned char buf[1024];
	int sel, len, i;
	int returnCode = 0;
	int wrote = 0;

	if(_debug)
		syslog(LOG_DEBUG, "is in %s\n", __FUNCTION__);

	wrote = write(fd, cmd, strlen(cmd));

	if(_debug)
		syslog(LOG_DEBUG, " wrote  %d \n", wrote);

	tcdrain(fd);
	sleep(1);
	//memset(buf, 0, sizeof(buf));
	//len = read(fd, buf, sizeof(buf));

	for (i = 0; i < 100; i++)
	{

		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		timeout.tv_sec = 0;
		timeout.tv_usec = to;

		if ((sel = select(fd + 1, &rfds, NULL, NULL, &timeout)) > 0)
		//if ((sel = select(fd + 1, &rfds, NULL, NULL, NULL)) > 0)
		{

			if (FD_ISSET(fd, &rfds))
			{
				memset(buf, 0, sizeof(buf));
				len = read(fd, buf, sizeof(buf));
				if(_debug)
					syslog(LOG_DEBUG, " read %d bytes == %s\n", len, buf);

				//if (strstr(buf, "\r\nOK\r\n") != NULL)
				if (strstr(buf, "OK") != NULL)
				{
					returnCode = 1;
					break;
				}
				if (strstr(buf, "ERROR") != NULL)
					break;
			}

		}

	}

	return returnCode;
}

/* Opens serial port, set's it to 57600bps 8N1 RTS/CTS mode.
*
* PARAMS:
* dev - device name
* RETURNS :
* file descriptor or -1 on error
*/
int open_serialport(char *dev)
{
	int fd;
	struct termios options;

	if(_debug)
		syslog(LOG_DEBUG, "is in %s\n" , __FUNCTION__);
	fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd != -1)
	{
		if(_debug)
			syslog(LOG_DEBUG, "serial opened\n" );
		fcntl(fd, F_SETFL, 0);

		// get the parameters
		tcgetattr(fd, &options);

		// Set the baud rates to 57600...
		// cfsetispeed(&options, B57600);
		// cfsetospeed(&options, B57600);

		// Enable the receiver and set local mode...
		options.c_cflag |= (CLOCAL | CREAD);

		// No parity (8N1):
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;

		// enable hardware flow control (CNEW_RTCCTS)
		// options.c_cflag |= CRTSCTS;

		// set raw input
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		// set raw output
		options.c_oflag &= ~OPOST;

		// Set the new options for the port...
		tcsetattr(fd, TCSANOW, &options);
	}
	return fd;
}

// Prints information on a frame
void print_frame(GSM0710_Frame * frame)
{
	if(_debug)
	{
		syslog(LOG_DEBUG, "is in %s\n" , __FUNCTION__);
		syslog(LOG_DEBUG,"Received ");
	}

	/**
	 * tooooooo looonnngggg to execute on
	 * embedded systems with limited resources
	 * and rigid time constraints
	 *
	if (FRAME_IS(SABM, frame))
	if(_debug)
			PDEBUG("SABM ");
	else if (FRAME_IS(UIH, frame))
     if(_debug)
			PDEBUG("UIH ");
	else if (FRAME_IS(UA, frame))
		PDEBUG("UA ");
	else if (FRAME_IS(DM, frame))
		PDEBUG("DM ");
	else if (FRAME_IS(DISC, frame))
		PDEBUG("DISC ");
	else if (FRAME_IS(UI, frame))
		PDEBUG("UI ");
	else
	{
		PDEBUG("unkown (control=%d) ", frame->control);
	}
	PDEBUG("frame for channel %d.\n", frame->channel);
	*/
    switch((frame->control & ~PF))
	{
		case SABM:
			if(_debug)
				syslog(LOG_DEBUG,"SABM ");
			break;
		case UIH:
			if(_debug)
				syslog(LOG_DEBUG,"UIH ");
			break;
		case UA:
			if(_debug)
				syslog(LOG_DEBUG,"UA ");
			break;
		case DM:
			if(_debug)
				syslog(LOG_DEBUG,"DM ");
			break;
		case DISC:
			if(_debug)
				syslog(LOG_DEBUG,"DISC ");
			break;
		case UI:
			if(_debug)
				syslog(LOG_DEBUG,"UI ");
			break;
		default:
			if(_debug)
				syslog(LOG_DEBUG,"unkown (control=%d) ", frame->control);
			break;
	}
	if(_debug)
		syslog(LOG_DEBUG," frame for channel %d.\n", frame->channel);

	if (frame->data_length > 0)
	{
		if(_debug)
		{
			syslog(LOG_DEBUG,"frame->data = %s / size = %d\n",frame->data, frame->data_length);
		//fwrite(frame->data, sizeof(char), frame->data_length, stdout);
			syslog(LOG_DEBUG,"\n");
		}
	}

}

/* Handles commands received from the control channel.
*/
void handle_command(GSM0710_Frame * frame)
{
#if 1
	unsigned char type, signals;
	int length = 0, i, type_length, channel, supported = 1;
	unsigned char *response;
	// struct ussp_operation op;

	if(_debug)
		syslog(LOG_DEBUG, "is in %s\n" , __FUNCTION__);

	if (frame->data_length > 0)
	{
		type = frame->data[0];  // only a byte long types are handled now
		// skip extra bytes
		for (i = 0; (frame->data_length > i && (frame->data[i] & EA) == 0); i++);
		i++;
		type_length = i;
		if ((type & CR) == CR)
		{
			// command not ack

			// extract frame length
			while (frame->data_length > i)
			{
				length = (length * 128) + ((frame->data[i] & 254) >> 1);
				if ((frame->data[i] & 1) == 1)
					break;
				i++;
			}
			i++;


			//i don't liked this too because this create tooooo lloooonnn lacks
			// handle commands
			/*
			if (COMMAND_IS(C_CLD, type))
			{
    if(_debug)
					PDEBUG("The mobile station requested mux-mode termination.\n");
				terminate = 1;
				terminateCount = -1;    // don't need to close down channels
			}
			else if (COMMAND_IS(C_TEST, type))
			{
#ifdef DEBUG
    if(_debug)
					PDEBUG("Test command: ");
    if(_debug)
					PDEBUG("frame->data = %s  / frame->data_length = %d",frame->data + i, frame->data_length - i);
				//fwrite(frame->data + i, sizeof(char), frame->data_length - i, stdout);
    if(_debug)
					PDEBUG("\n");
#endif
			}
			else if (COMMAND_IS(C_MSC, type))
			{
				if (i + 1 < frame->data_length)
				{
					channel = ((frame->data[i] & 252) >> 2);
					i++;
					signals = (frame->data[i]);
					// op.op = USSP_MSC;
					// op.arg = USSP_RTS;
					// op.len = 0;

     if(_debug)
						PDEBUG("Modem status command on channel %d.\n", channel);
					if ((signals & S_FC) == S_FC)
					{
      if(_debug)
							PDEBUG("No frames allowed.\n");
					}
					else
					{
						// op.arg |= USSP_CTS;
      if(_debug)
							PDEBUG("Frames allowed.\n");
					}
					if ((signals & S_RTC) == S_RTC)
					{
						// op.arg |= USSP_DSR;
      if(_debug)
							PDEBUG("RTC\n");
					}
					if ((signals & S_IC) == S_IC)
					{
						// op.arg |= USSP_RI;
      if(_debug)
							PDEBUG("Ring\n");
					}
					if ((signals & S_DV) == S_DV)
					{
						// op.arg |= USSP_DCD;
      if(_debug)
							PDEBUG("DV\n");
					}
					// if (channel > 0)
					//     write(ussp_fd[(channel - 1)], &op, sizeof(op));
				}
				else
				{
     if(_debug)
						PDEBUG("ERROR: Modem status command, but no info. i: %d, len: %d, data-len: %d\n", i, length,
						frame->data_length);
				}
			}
			else
			{
    if(_debug)
					PDEBUG("Unknown command (%d) from the control channel.\n", type);
				response = malloc(sizeof(char) * (2 + type_length));
				response[0] = C_NSC;
				// supposes that type length is less than 128
				response[1] = EA & ((127 & type_length) << 1);
				i = 2;
				while (type_length--)
				{
					response[i] = frame->data[(i - 2)];
					i++;
				}
				write_frame(0, response, i, UIH);
				free(response);
				supported = 0;
			}
			*/
			switch((type & ~CR))
			{
				case C_CLD:
					if(_debug)
						syslog(LOG_DEBUG,"The mobile station requested mux-mode termination.\n");
					terminate = 1;
					terminateCount = -1;    // don't need to close down channels
					break;
				case C_TEST:
	#ifdef DEBUG
					if(_debug)
						syslog(LOG_DEBUG,"Test command: ");
					if(_debug)
						syslog(LOG_DEBUG,"frame->data = %s  / frame->data_length = %d\n",frame->data + i, frame->data_length - i);
					//fwrite(frame->data + i, sizeof(char), frame->data_length - i, stdout);
	#endif
					break;
				case C_MSC:
					if (i + 1 < frame->data_length)
					{
						channel = ((frame->data[i] & 252) >> 2);
						i++;
						signals = (frame->data[i]);
						// op.op = USSP_MSC;
						// op.arg = USSP_RTS;
						// op.len = 0;

						if(_debug)
							syslog(LOG_DEBUG,"Modem status command on channel %d.\n", channel);
						if ((signals & S_FC) == S_FC)
						{
							if(_debug)
								syslog(LOG_DEBUG,"No frames allowed.\n");
						}
						else
						{
							// op.arg |= USSP_CTS;
							if(_debug)
								syslog(LOG_DEBUG,"Frames allowed.\n");
						}
						if ((signals & S_RTC) == S_RTC)
						{
							// op.arg |= USSP_DSR;
							if(_debug)
								syslog(LOG_DEBUG,"RTC\n");
						}
						if ((signals & S_IC) == S_IC)
						{
							// op.arg |= USSP_RI;
							if(_debug)
								syslog(LOG_DEBUG,"Ring\n");
						}
						if ((signals & S_DV) == S_DV)
						{
							// op.arg |= USSP_DCD;
							if(_debug)
								syslog(LOG_DEBUG,"DV\n");
						}
						// if (channel > 0)
						//     write(ussp_fd[(channel - 1)], &op, sizeof(op));
					}
					else
					{
						syslog(LOG_ERR,"ERROR: Modem status command, but no info. i: %d, len: %d, data-len: %d\n", i, length,
						frame->data_length);
					}
					break;
				default:
					syslog(LOG_ALERT,"Unknown command (%d) from the control channel.\n", type);
					response = malloc(sizeof(char) * (2 + type_length));
					response[0] = C_NSC;
					// supposes that type length is less than 128
					response[1] = EA & ((127 & type_length) << 1);
					i = 2;
					while (type_length--)
					{
						response[i] = frame->data[(i - 2)];
						i++;
					}
					write_frame(0, response, i, UIH);
					free(response);
					supported = 0;
					break;
			}

			if (supported)
			{
				// acknowledge the command
				frame->data[0] = frame->data[0] & ~CR;
				write_frame(0, frame->data, frame->data_length, UIH);
			}
		}
		else
		{
			// received ack for a command
			if (COMMAND_IS(C_NSC, type))
			{
				syslog(LOG_ALERT,"The mobile station didn't support the command sent.\n");
			}
			else
			{
				if(_debug)
					syslog(LOG_DEBUG,"Command acknowledged by the mobile station.\n");
			}
		}
	}
#endif
}

// shows how to use this program
void usage(char *_name)
{
	fprintf(stderr,"\nUsage: %s [options] <pty1> <pty2> ...\n",_name);
	fprintf(stderr,"  <ptyN>          : pty devices (e.g. /dev/ptya0)\n\n");
	fprintf(stderr,"options:\n");
	fprintf(stderr,"  -p <serport>    : Serial port device to connect to [/dev/modem]\n");
	fprintf(stderr,"  -f <framsize>   : Maximum frame size [32]\n");
	fprintf(stderr,"  -d              : Debug mode, don't fork\n");
	fprintf(stderr,"  -m <modem>      : Modem (mc35, generic, ...)\n");
	fprintf(stderr,"  -h              : Show this help message\n");
}

/* Extracts and handles frames from the receiver buffer.
*
* PARAMS:
* buf - the receiver buffer
*/
void extract_frames(GSM0710_Buffer * buf)
{
	// version test for Siemens terminals to enable version 2 functions
	static char version_test[] = "\x23\x21\x04TEMUXVERSION2\0\0";

	GSM0710_Frame *frame;

	if(_debug)
		syslog(LOG_DEBUG, "is in %s\n" , __FUNCTION__);

	while ((frame = gsm0710_buffer_get_frame(buf)))
	{
		if ((FRAME_IS(UI, frame) || FRAME_IS(UIH, frame)))
		{
			if(_debug)
				syslog(LOG_DEBUG, "is (FRAME_IS(UI, frame) || FRAME_IS(UIH, frame))\n");
			if (frame->channel > 0)
			{
				if(_debug)
					syslog(LOG_DEBUG,"frame->channel > 0\n");
				// data from logical channel
				ussp_send_data(frame->data, frame->data_length, frame->channel - 1);
			}
			else
			{
				// control channel command
				if(_debug)
					syslog(LOG_DEBUG,"control channel command\n");
				handle_command(frame);
			}
		}
		else
		{
			// not an information frame
			if(_debug)
				syslog(LOG_DEBUG,"not an information frame\n");
#ifdef DEBUG
			print_frame(frame);
#endif
			/*
			if (FRAME_IS(UA, frame))
			{
    if(_debug)
					PDEBUG("is FRAME_IS\(UA, frame\)\n");
				if (cstatus[frame->channel].opened == 1)
				{
     if(_debug)
						PDEBUG("Logical channel %d closed.\n", frame->channel);
					cstatus[frame->channel].opened = 0;
				}
				else
				{
					cstatus[frame->channel].opened = 1;
					if (frame->channel == 0)
					{
      if(_debug)
							PDEBUG("Control channel opened.\n");
						// send version Siemens version test
						write_frame(0, version_test, 18, UIH);
					}
					else
					{
      if(_debug)
							PDEBUG("Logical channel %d opened.\n", frame->channel);
					}
				}
			}
			else if (FRAME_IS(DM, frame))
			{
				if (cstatus[frame->channel].opened)
				{
     if(_debug)
						PDEBUG("DM received, so the channel %d was already closed.\n", frame->channel);
					cstatus[frame->channel].opened = 0;
				}
				else
				{
					if (frame->channel == 0)
					{
      if(_debug)
							PDEBUG("Couldn't open control channel.\n->Terminating.\n");
						terminate = 1;
						terminateCount = -1;    // don't need to close channels
					}
					else
					{
      if(_debug)
							PDEBUG("Logical channel %d couldn't be opened.\n", frame->channel);
					}
				}
			}
			else if (FRAME_IS(DISC, frame))
			{
				// channel close request
				if (cstatus[frame->channel].opened)
				{
					cstatus[frame->channel].opened = 0;
					write_frame(frame->channel, NULL, 0, UA | PF);
					if (frame->channel == 0)
					{
      if(_debug)
							PDEBUG("Control channel closed.\n");
						terminate = 1;
						terminateCount = -1;    // don't need to close channels
					}
					else
					{
      if(_debug)
							PDEBUG("Logical channel %d closed.\n", frame->channel);
					}
				}
				else
				{
					// channel already closed
     if(_debug)
						PDEBUG("Received DISC even though channel %d was already closed.\n", frame->channel);
					write_frame(frame->channel, NULL, 0, DM | PF);
				}
			}
			else if (FRAME_IS(SABM, frame))
			{
				// channel open request
				if (cstatus[frame->channel].opened == 0)
				{
					if (frame->channel == 0)
					{
      if(_debug)
							PDEBUG("Control channel opened.\n");
					}
					else
					{
      if(_debug)
							PDEBUG("Logical channel %d opened.\n", frame->channel);
					}
				}
				else
				{
					// channel already opened
     if(_debug)
						PDEBUG("Received SABM even though channel %d was already closed.\n", frame->channel);
				}
				cstatus[frame->channel].opened = 1;
				write_frame(frame->channel, NULL, 0, UA | PF);
			}
			*/
            switch((frame->control & ~PF))
			{
				case UA:
					if(_debug)
						syslog(LOG_DEBUG,"is FRAME_IS(UA, frame)\n");
					if (cstatus[frame->channel].opened == 1)
					{
						syslog(LOG_INFO,"Logical channel %d closed.\n", frame->channel);
						cstatus[frame->channel].opened = 0;
					}
					else
					{
						cstatus[frame->channel].opened = 1;
						if (frame->channel == 0)
						{
							syslog(LOG_INFO,"Control channel opened.\n");
							// send version Siemens version test
							write_frame(0, version_test, 18, UIH);
						}
						else
						{
							syslog(LOG_INFO,"Logical channel %d opened.\n", frame->channel);
						}
					}
					break;
				case DM:
					if (cstatus[frame->channel].opened)
					{
						syslog(LOG_INFO,"DM received, so the channel %d was already closed.\n", frame->channel);
						cstatus[frame->channel].opened = 0;
					}
					else
					{
						if (frame->channel == 0)
						{
							syslog(LOG_INFO,"Couldn't open control channel.\n->Terminating.\n");
							terminate = 1;
							terminateCount = -1;    // don't need to close channels
						}
						else
						{
							syslog(LOG_INFO,"Logical channel %d couldn't be opened.\n", frame->channel);
						}
					}
					break;
				case DISC:
					if (cstatus[frame->channel].opened)
					{
						cstatus[frame->channel].opened = 0;
						write_frame(frame->channel, NULL, 0, UA | PF);
						if (frame->channel == 0)
						{
							syslog(LOG_INFO,"Control channel closed.\n");
							terminate = 1;
							terminateCount = -1;    // don't need to close channels
						}
						else
						{
							syslog(LOG_INFO,"Logical channel %d closed.\n", frame->channel);
						}
					}
					else
					{
						// channel already closed
						syslog(LOG_INFO,"Received DISC even though channel %d was already closed.\n", frame->channel);
						write_frame(frame->channel, NULL, 0, DM | PF);
					}
					break;
				case SABM:
					// channel open request
					if (cstatus[frame->channel].opened == 0)
					{
						if (frame->channel == 0)
						{
							syslog(LOG_INFO,"Control channel opened.\n");
						}
						else
						{
							syslog(LOG_INFO,"Logical channel %d opened.\n", frame->channel);
						}
					}
					else
					{
						// channel already opened
						syslog(LOG_INFO,"Received SABM even though channel %d was already closed.\n", frame->channel);
					}
					cstatus[frame->channel].opened = 1;
					write_frame(frame->channel, NULL, 0, UA | PF);
					break;
			}
		}

		destroy_frame(frame);
	}
	if(_debug)
		syslog(LOG_DEBUG,"out of %s\n", __FUNCTION__);
}



/**
 * Daemonize process, this process  create teh daemon
 */
int daemonize(int _debug)
{
	int maxi;
	if(!_debug)
	{
		if((the_pid=fork()) < 0)
			return(-1);
		else
			if(the_pid!=0)
				exit(0);//parent goes bye-bye
		//child continues
		setsid();   //become session leader
		//signal(SIGHUP, SIG_IGN);
		if((the_pid = fork()) != 0)
			exit(0);
		chdir("/"); //change working directory
		umask(0);// clear our file mode creation mask
	}
	//daemonize process stop here
	/*close all the file descriptors in the system
	daemonize yet
	*/
	fprintf(stdout, "Daemon starting, look syslog messages....\n");

	//close all file descriptors
	for(maxi = 0; maxi < 64; maxi++) close(maxi);

	return 0;
}

/**
 * Function responsible by all signal handlers treatment
 * any new signal must be added here
 */
void signal_treatment(int param)
{
	switch(param)
	{
		case SIGPIPE:
			exit(0);
			break;
		case SIGHUP:
			//reread the configuration files
			break;
		case SIGINT:
			//exit(0);
			terminate = 1;
			break;
		case SIGKILL:
			//kill immediatly
			//i'm not sure if i put exit or sustain the terminate attribution
			terminate = 1;
			//exit(0);
			break;
		case SIGUSR1:
			terminate  = 1;
			//sig_term(param);
		case SIGTERM:
			terminate = 1;
			break;
		default:
			exit(0);
			break;
	}

}

/**
 * Fuunction to init Modemd Siemes MC35 families
 * Siemens need and special step-by for after get-in MUX state
 */
void initSiemensMC35()
{
	char mux_command[] = "AT+CMUX=0\r\n";
	//Modem Init for Siemens MC35i
	if (!at_command(serial_fd,"AT\r\n", 10000))
	{
		if(_debug)
			syslog(LOG_DEBUG, "ERROR AT %d\r\n", __LINE__);
	}

	if (!at_command(serial_fd,"AT+IPR=57600\r\n", 10000))
	{
		if(_debug)
			syslog(LOG_DEBUG, "ERROR AT+IPR=57600 %d \r\n", __LINE__);
	}
	if (!at_command(serial_fd,"AT\r\n", 10000))
	{
		if(_debug)
			syslog(LOG_DEBUG, "ERROR AT %d \r\n", __LINE__);
	}

	if (!at_command(serial_fd,"AT&S0\r\n", 10000))
	{
		if(_debug)
			syslog(LOG_DEBUG, "ERRO AT&S0 %d\r\n", __LINE__);
	}
	if (!at_command(serial_fd,"AT\\Q3\r\n", 10000))
	{
		if(_debug)
			syslog(LOG_DEBUG, "ERRO AT\\Q3 %d\r\n", __LINE__);
	}
	if (!at_command(serial_fd, mux_command, 10000))
	{
		syslog(LOG_ERR, "MUX mode doesn't function.\n");
		exit(-1);
	}
}

/**
 * Function to start modems that only needs at+cmux=X to get-in mux state
 */
void initGeneric()
{
	char mux_command[] = "AT+CMUX=0\r\n";
	/**
	 * Modem Init for Siemens Generic like Sony
	 * that don't need initialization sequence like Siemens MC35
	 */
	if (!at_command(serial_fd,"AT\r\n", 10000))
	{
		if(_debug)
			syslog(LOG_DEBUG, "ERROR AT %d\r\n", __LINE__);
	}

	if (!at_command(serial_fd, mux_command, 10000))
	{
		syslog(LOG_ERR, "MUX mode doesn't function.\n");
		exit(-1);
	}
}



/**
 * The main program
 */
int main(int argc, char *argv[], char *env[])
{
	//struct sigaction sa;
	int sel, len, maxfd;
	fd_set rfds;
	struct timeval timeout;
	unsigned char buf[4096], **tmp;
	int *remaining;
	int first_ttyU = 0;
	char *programName;
	int i, size,t;
	int numOfPorts;

	unsigned char close_mux[2] = { C_CLD | CR, 1 };
	int opt;
	char *serportdev;
	char *ptydev[MAX_CHANNELS];


	programName = argv[0];
	/*************************************/
	if(argc<2)
	{
		usage(programName);
		exit(-1);
	}
	_modem_type = GENERIC;

	serportdev="/dev/modem";

	while((opt=getopt(argc,argv,"p:f:h:d:m"))>0)
	{
		switch(opt)
		{
			case 'p' :
				serportdev = optarg;
				break;
			case 'f' :
				max_frame_size = atoi(optarg);
				break;
			//Vitorio
			case 'd' :
				_debug = 1;
				break;
			case 'm':
				if(!strcmp(optarg,"mc35"))
					_modem_type = MC35;
				else if(!strcmp(optarg,"generic"))
						_modem_type = GENERIC;
					else _modem_type = UNKNOW_MODEM;
				break;
			case '?' :
			case 'h' :
				usage(programName);
				exit(0);
				break;
			default:
				break;
		}
	}
	//DAEMONIZE
	//SHOW TIME

	daemonize(_debug);
	//The Hell is from now-one

    /* SIGNALS treatment*/
	signal(SIGHUP, signal_treatment);
	signal(SIGPIPE, signal_treatment);
	signal(SIGKILL, signal_treatment);
	signal(SIGINT, signal_treatment);
    signal(SIGUSR1, signal_treatment);
	signal(SIGTERM, signal_treatment);

	programName = argv[0];
	if(_debug)
	{
		openlog(programName, LOG_NDELAY | LOG_PID | LOG_PERROR  , LOG_LOCAL0);//pode ir até 7
		_priority = LOG_DEBUG;
	}
	else
	{
		openlog(programName, LOG_NDELAY | LOG_PID , LOG_LOCAL0 );//pode ir até 7
		_priority = LOG_INFO;
	}



	for(t=optind;t<argc;t++)
	{
		if((t-optind)>=MAX_CHANNELS) break;
		syslog(LOG_INFO, "Port %d : %s\n",t-optind,argv[t]);
		ptydev[t-optind]=argv[t];
	}
	//exit(0);
	numOfPorts = t-optind;

	syslog(LOG_INFO,"Malloc buffers...\n");
	// allocate memory for data structures
	if (!(ussp_fd = malloc(sizeof(int) * numOfPorts))
		|| !(in_buf = gsm0710_buffer_init())
		|| !(remaining = malloc(sizeof(int) * numOfPorts))
		|| !(tmp = malloc(sizeof(char *) * numOfPorts))
		|| !(cstatus = malloc(sizeof(Channel_Status) * (1 + numOfPorts))))
	{
		syslog(LOG_ALERT,"Out of memory\n");
		exit(-1);
	}
	syslog(LOG_INFO,"Open devices...\n");
	// open ussp devices
	maxfd = 0;
	for (i = 0; i < numOfPorts; i++)
	{
		remaining[i] = 0;
		if ((ussp_fd[i] = open(ptydev[i], O_RDWR | O_NONBLOCK)) < 0)
		{
			syslog(LOG_ERR,"Can't open %s. %s (%d).\n", ptydev[i], strerror(errno), errno);
			exit(-1);
		}
		else if (ussp_fd[i] > maxfd)
			maxfd = ussp_fd[i];
		cstatus[i].opened = 0;
		cstatus[i].v24_signals = S_DV | S_RTR | S_RTC | EA;
	}
	cstatus[i].opened = 0;
	terminateCount = numOfPorts;
	syslog(LOG_INFO,"Open serial port...\n");

	// open the serial port
	if ((serial_fd = open_serialport(serportdev)) < 0)
	{
		syslog(LOG_ALERT,"Can't open %s. %s (%d).\n", serportdev, strerror(errno), errno);
		return -1;
	}
	else if (serial_fd > maxfd)
		maxfd = serial_fd;

	syslog(LOG_INFO,"Opened serial port. Switching to mux-mode.\n");

	/**
	 * heere we muste agree with some method of hardware initialization
	 * the following lines is for Siemens MC35i and are according Developers Device Drivers Manual
	 * from Siemens because this i put inside an IFDEF until we agree
	 * in a final solution
	 */

	switch(_modem_type)
	{
		case MC35:
			initSiemensMC35(); //we coould have other models like XP48 TC45/35
			break;
		case GENERIC:
			initGeneric();
			break;
		//case default:
		//	syslog(LOG_ERR, "OOPS Strange modem\n");
		//	break;
	}

	//End Modem Init

	syslog(LOG_INFO, "Waiting for mux-mode.\n");
	sleep(1);
	syslog(LOG_INFO, "Opening control channel.\n");
	write_frame(0, NULL, 0, SABM | PF);
	syslog(LOG_INFO, "Opening logical channels.\n");
	for (i = 1; i <= numOfPorts; i++)
	{
		sleep(1);
		write_frame(i, NULL, 0, SABM | PF);
	}
	syslog(LOG_INFO, "Connecting /dev/ttya[%d-%d] to %s\n", first_ttyU, (first_ttyU + numOfPorts - 1), argv[1]);
	if(_debug)
	syslog(LOG_INFO, "You can quit the MUX daemon with SIGKILL or SIGTERM\n");

	/**
	 * SUGGESTION:
	 * substitute this lack for two threads
	 */
	// -- start waiting for input and forwarding it back and forth --
	while (!terminate || terminateCount >= -1)
	{

		FD_ZERO(&rfds);
		FD_SET(serial_fd, &rfds);
		for (i = 0; i < numOfPorts; i++)
			FD_SET(ussp_fd[i], &rfds);

		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		if ((sel = select(maxfd + 1, &rfds, NULL, NULL, &timeout)) > 0)
		{

			if (FD_ISSET(serial_fd, &rfds))
			{
				// input from serial port
				if(_debug)
					syslog(LOG_DEBUG, "Serial Data\n");
				if ((size = gsm0710_buffer_free(in_buf)) > 0 && (len = read(serial_fd, buf, min(size, sizeof(buf)))) > 0)
				{
					gsm0710_buffer_write(in_buf, buf, len);
					// extract and handle ready frames
					extract_frames(in_buf);
				}
			}


			// check virtual ports
			for (i = 0; i < numOfPorts; i++)
				if (FD_ISSET(ussp_fd[i], &rfds))
				{

					// information from virtual port
					if (remaining[i] > 0)
					{
						memcpy(buf, tmp[i], remaining[i]);
						free(tmp[i]);
					}
					if ((len = read(ussp_fd[i], buf + remaining[i], sizeof(buf) - remaining[i])) > 0)
						remaining[i] = ussp_recv_data(buf, len + remaining[i], i);
					if(_debug)
						syslog(LOG_DEBUG,"Data from ptya%d: %d bytes\n",i,len);
					if(len<0)
					{
						remaining[i] = 0;
						close(ussp_fd[i]);
						if ((ussp_fd[i] = open(ptydev[i], O_RDWR | O_NONBLOCK)) < 0)
						{
							if(_debug)
								syslog(LOG_DEBUG,"Can't re-open %s. %s (%d).\n", ptydev[i], strerror(errno), errno);
							terminate=1;
						}
						else if (ussp_fd[i] > maxfd)
							maxfd = ussp_fd[i];
					}

					/* copy remaining bytes from last packet into tmp */
					if (remaining[i] > 0)
					{
						tmp[i] = malloc(remaining[i]);
						memcpy(tmp[i], buf + sizeof(buf) - remaining[i], remaining[i]);
					}
				}
		}

		if (terminate)
		{
			// terminate command given. Close channels one by one and finaly
			// close the mux mode

			if (terminateCount > 0)
			{
				syslog(LOG_INFO,"Closing down the logical channel %d.\n", terminateCount);
				if (cstatus[terminateCount].opened)
					write_frame(terminateCount, NULL, 0, DISC | PF);
			}
			else if (terminateCount == 0)
			{
				syslog(LOG_INFO,"Sending close down request to the multiplexer.\n");
				write_frame(0, close_mux, 2, UIH);
			}
			terminateCount--;
		}

	}                           /* while */

	// finalize everything

	close(serial_fd);

	for (i = 0; i < numOfPorts; i++)
		close(ussp_fd[i]);
	free(ussp_fd);
	free(tmp);
	free(remaining);
	syslog(LOG_INFO,"Received %ld frames and dropped %ld received frames during the mux-mode.\n", in_buf->received_count,
		in_buf->dropped_count);
	gsm0710_buffer_destroy(in_buf);
	syslog(LOG_INFO, "%s finished\n", programName);
	/**
	 * close  syslog
	 */
	closelog();
	return 0;

}
