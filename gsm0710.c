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
 *  -h              : Show this help message
 */

#define DEBUG 1

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
#include <malloc.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "buffer.h"
#include "gsm0710.h"

#define DEFAULT_NUMBER_OF_PORTS 3
#define WRITE_RETRIES 5
#define MAX_CHANNELS   32

static volatile int terminate = 0;
static int terminateCount = 0;

static int *ussp_fd;
static int serial_fd;
static Channel_Status *cstatus;
static int max_frame_size = 31; // The limit of Sony-Ericsson GM47

static GSM0710_Buffer *in_buf;  // input buffer

static void sig_usr1(int sig)
{
}

static void sig_term(int sig)
{
    printf("CTRL-C\n");
    terminate = 1;
}

static void sig_hup(int sig)
{
}

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

/* Returns success, when an ussp is opened.
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

    PDEBUG("send frame to ch %d :", channel);
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
        PDEBUG("Couldn't write the whole prefix to the serial port for the virtual port %d. Wrote only %d  bytes.",
               channel, c);
        return 0;
    }
    if (count > 0)
    {
        c = write(serial_fd, input, count);
        if (count != c)
        {
            PDEBUG("Couldn't write all data to the serial port from the virtual port %d. Wrote only %d bytes.\n",
                   channel, c);
            return 0;
        }
    }
    c = write(serial_fd, postfix, 2);
    if (c != 2)
    {
        PDEBUG("Couldn't write the whole postfix to the serial port for the virtual port %d. Wrote only %d bytes.",
               channel, c);
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
                printf("Set termios for ussp port %d\n", port);
                printf("\tinput mode flags:   0x%04x\n", tiosp->c_iflag);
                printf("\toutput mode flags:  0x%04x\n", tiosp->c_oflag);
                printf("\tcontrol mode flags: 0x%04x\n", tiosp->c_cflag);
                printf("\tlocal mode flags:   0x%04x\n", tiosp->c_lflag);
                printf("\tline discipline:    0x%02x\n", tiosp->c_line);
                printf("\tcontrol characters: ");
                for (i = 0; i < NCCS; i++)
                    printf("0x%02x ", tiosp->c_cc[i]);
                printf("\n");
                printf("\tinput speed:        0x%02x (%i)\n", tiosp->c_ispeed, tiosp->c_ispeed);
                printf("\toutput speed:       0x%02x (%i)\n", tiosp->c_ospeed, tiosp->c_ospeed);
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
        PDEBUG("Couldn't write data to channel %d. Wrote only %d bytes, when should have written %ld.\n",
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
    PDEBUG("send data to port ptya%d\n", port);
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

    write(fd, cmd, strlen(cmd));
    tcdrain(fd);

    for (i = 0; i < 100; i++)
    {

        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        timeout.tv_sec = 0;
        timeout.tv_usec = to;

        if ((sel = select(fd + 1, &rfds, NULL, NULL, &timeout)) > 0)
        {

            if (FD_ISSET(fd, &rfds))
            {
                memset(buf, 0, sizeof(buf));
                len = read(fd, buf, sizeof(buf));

                if (strstr(buf, "\r\nOK\r\n") != NULL)
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

    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd != -1)
    {
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
    printf("Received ");
    if (FRAME_IS(SABM, frame))
        printf("SABM ");
    else if (FRAME_IS(UIH, frame))
        printf("UIH ");
    else if (FRAME_IS(UA, frame))
        printf("UA ");
    else if (FRAME_IS(DM, frame))
        printf("DM ");
    else if (FRAME_IS(DISC, frame))
        printf("DISC ");
    else if (FRAME_IS(UI, frame))
        printf("UI ");
    else
    {
        printf("unkown (control=%d) ", frame->control);
    }
    printf("frame for channel %d.\n", frame->channel);
    if (frame->data_length > 0)
    {
        fwrite(frame->data, sizeof(char), frame->data_length, stdout);
        printf("\n");
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

            // handle commands
            if (COMMAND_IS(C_CLD, type))
            {
                printf("The mobile station requested mux-mode termination.\n");
                terminate = 1;
                terminateCount = -1;    // don't need to close down channels
            }
            else if (COMMAND_IS(C_TEST, type))
            {
#ifdef DEBUG
                printf("Test command: ");
                fwrite(frame->data + i, sizeof(char), frame->data_length - i, stdout);
                printf("\n");
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

                    PDEBUG("Modem status command on channel %d.\n", channel);
                    if ((signals & S_FC) == S_FC)
                    {
                        PDEBUG("No frames allowed.\n");
                    }
                    else
                    {
                        // op.arg |= USSP_CTS;
                        PDEBUG("Frames allowed.\n");
                    }
                    if ((signals & S_RTC) == S_RTC)
                    {
                        // op.arg |= USSP_DSR;
                        PDEBUG("RTC\n");
                    }
                    if ((signals & S_IC) == S_IC)
                    {
                        // op.arg |= USSP_RI;
                        PDEBUG("Ring\n");
                    }
                    if ((signals & S_DV) == S_DV)
                    {
                        // op.arg |= USSP_DCD;
                        PDEBUG("DV\n");
                    }
                    // if (channel > 0)
                    //     write(ussp_fd[(channel - 1)], &op, sizeof(op));
                }
                else
                {
                    PDEBUG("ERROR: Modem status command, but no info. i: %d, len: %d, data-len: %d\n", i, length,
                           frame->data_length);
                }
            }
            else
            {
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
                PDEBUG("The mobile station didn't support the command sent.\n");
            }
            else
            {
                PDEBUG("Command acknowledged by the mobile station.\n");
            }
        }
    }
#endif
}

// shows how to use this program
void usage()
{
    printf("\nUsage: gsm0710 [options] <pty1> <pty2> ...\n");
    printf("  <ptyN>          : pty devicaes (e.g. /dev/ptya0)\n\n");
    printf("options:\n");
    printf("  -p <serport>    : Serial port device to connect to [/dev/modem]\n");
    printf("  -f <framsize>   : Maximum frame size [32]\n");
    printf("  -h              : Show this help message\n");
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

    while ((frame = gsm0710_buffer_get_frame(buf)))
    {
        if ((FRAME_IS(UI, frame) || FRAME_IS(UIH, frame)))
        {
            if (frame->channel > 0)
            {
                // data from logical channel
                ussp_send_data(frame->data, frame->data_length, frame->channel - 1);
            }
            else
            {
                // control channel command
                handle_command(frame);
            }
        }
        else
        {
            // not an information frame
#ifdef DEBUG
            print_frame(frame);
#endif
            if (FRAME_IS(UA, frame))
            {
                if (cstatus[frame->channel].opened == 1)
                {
                    printf("Logical channel %d closed.\n", frame->channel);
                    cstatus[frame->channel].opened = 0;
                }
                else
                {
                    cstatus[frame->channel].opened = 1;
                    if (frame->channel == 0)
                    {
                        printf("Control channel opened.\n");
                        // send version Siemens version test
                        write_frame(0, version_test, 18, UIH);
                    }
                    else
                    {
                        printf("Logical channel %d opened.\n", frame->channel);
                    }
                }
            }
            else if (FRAME_IS(DM, frame))
            {
                if (cstatus[frame->channel].opened)
                {
                    printf("DM received, so the channel %d was already closed.\n", frame->channel);
                    cstatus[frame->channel].opened = 0;
                }
                else
                {
                    if (frame->channel == 0)
                    {
                        printf("Couldn't open control channel.\n->Terminating.\n");
                        terminate = 1;
                        terminateCount = -1;    // don't need to close channels
                    }
                    else
                    {
                        printf("Logical channel %d couldn't be opened.\n", frame->channel);
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
                        PDEBUG("Control channel closed.\n");
                        terminate = 1;
                        terminateCount = -1;    // don't need to close channels
                    }
                    else
                    {
                        PDEBUG("Logical channel %d closed.\n", frame->channel);
                    }
                }
                else
                {
                    // channel already closed
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
                        printf("Control channel opened.\n");
                    }
                    else
                    {
                        printf("Logical channel %d opened.\n", frame->channel);
                    }
                }
                else
                {
                    // channel already opened
                    PDEBUG("Received SABM even though channel %d was already closed.\n", frame->channel);
                }
                cstatus[frame->channel].opened = 1;
                write_frame(frame->channel, NULL, 0, UA | PF);
            }
        }
        destroy_frame(frame);
    }

}


// the main program
int main(int argc, char *argv[], char *env[])
{
    struct sigaction sa;

    int sel, len, maxfd;
    fd_set rfds;
    struct timeval timeout;
    unsigned char buf[4096], **tmp;
    int *remaining;
    int first_ttyU = 0;

    int i, size,t;
    int numOfPorts;
    char mux_command[] = "AT+CMUX=0,0,5\r\n";
    unsigned char close_mux[2] = { C_CLD | CR, 1 };

    int opt;
    char *serportdev;
    char *ptydev[MAX_CHANNELS];
    
    if(argc<2)
    {
        usage();
        exit(-1);
    }
    
    serportdev="/dev/modem";
    while((opt=getopt(argc,argv,"p:f:h"))>0)
    {
        switch(opt)
        {
            case 'p' : 
                serportdev = optarg;
                break;
            case 'f' :
                max_frame_size = atoi(optarg);
                break;
            case '?' :
            case 'h' :
                usage();
                exit(0);
                break;
            default:
                break;
        }
    }
    for(t=optind;t<argc;t++)
    {
        if((t-optind)>=MAX_CHANNELS) break;
        PDEBUG("Port %d : %s\n",t-optind,argv[t]);
        ptydev[t-optind]=argv[t];
    }
    //exit(0);
    numOfPorts = t-optind;
    
    // set signal handler
    memset(&sa, 0, sizeof(sa));
    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = sig_term;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sa.sa_handler = sig_hup;
    sigaction(SIGHUP, &sa, NULL);
    sa.sa_handler = sig_usr1;
    sigaction(SIGUSR1, &sa, NULL);
    sa.sa_handler = SIG_IGN;
    sigaction(SIGCHLD, &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);
    printf("Malloc buffers...\n");
    // allocate memory for data structures
    if (!(ussp_fd = malloc(sizeof(int) * numOfPorts))
        || !(in_buf = gsm0710_buffer_init())
        || !(remaining = malloc(sizeof(int) * numOfPorts))
        || !(tmp = malloc(sizeof(char *) * numOfPorts))
        || !(cstatus = malloc(sizeof(Channel_Status) * (1 + numOfPorts))))
    {
        printf("Out of memory\n");
        return -1;
    }
    PDEBUG("Open devices...\n");
    // open ussp devices
    maxfd = 0;
    for (i = 0; i < numOfPorts; i++)
    {
        remaining[i] = 0;
        if ((ussp_fd[i] = open(ptydev[i], O_RDWR | O_NONBLOCK)) < 0)
        {
            printf("Can't open %s. %s (%d).\n", ptydev[i], strerror(errno), errno);
            exit(-1);
        }
        else if (ussp_fd[i] > maxfd)
            maxfd = ussp_fd[i];
        cstatus[i].opened = 0;
        cstatus[i].v24_signals = S_DV | S_RTR | S_RTC | EA;
    }
    cstatus[i].opened = 0;
    terminateCount = numOfPorts;
    PDEBUG("Open serial port...\n");

    // open the serial port
    if ((serial_fd = open_serialport(serportdev)) < 0)
    {
        printf("Can't open %s. %s (%d).\n", serportdev, strerror(errno), errno);
        return -1;
    }
    else if (serial_fd > maxfd)
        maxfd = serial_fd;

    PDEBUG("Opened serial port. Switching to mux-mode.\n");
    at_command(serial_fd, "AT\r\n", 10000);
    if (!at_command(serial_fd, mux_command, 10000))
    {
        printf("MUX mode doesn't function.\n");
        return -1;
    }
    PDEBUG("Waiting for mux-mode.\n");
    sleep(1);
    PDEBUG("Opening control channel.\n");
    write_frame(0, NULL, 0, SABM | PF);
    PDEBUG("Opening logical channels.\n");
    for (i = 1; i <= numOfPorts; i++)
    {
        sleep(1);
        write_frame(i, NULL, 0, SABM | PF);
    }
    printf("Connecting /dev/ttyU[%d-%d] to %s\n", first_ttyU, (first_ttyU + numOfPorts - 1), argv[1]);
    printf("You can quit the MUX daemon with CTRL-C.\n");

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
                PDEBUG("Serial Data\n");
                if ((size = gsm0710_buffer_free(in_buf)) > 0 &&
                    (len = read(serial_fd, buf, min(size, sizeof(buf)))) > 0)
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
                    PDEBUG("Data from ptya%d: %d bytes\n",i,len);
                    if(len<0) 
                    {
                        remaining[i] = 0;
                        close(ussp_fd[i]);
                        if ((ussp_fd[i] = open(ptydev[i], O_RDWR | O_NONBLOCK)) < 0)
                        {
                            printf("Can't re-open %s. %s (%d).\n", ptydev[i], strerror(errno), errno);
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
                PDEBUG("Closing down the logical channel %d.\n", terminateCount);
                if (cstatus[terminateCount].opened)
                    write_frame(terminateCount, NULL, 0, DISC | PF);
            }
            else if (terminateCount == 0)
            {
                PDEBUG("Sending close down request to the multiplexer.\n");
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
    printf("Received %ld frames and dropped %ld received frames during the mux-mode.\n", in_buf->received_count,
           in_buf->dropped_count);
    gsm0710_buffer_destroy(in_buf);

    return 0;

}
