#include "serial_util.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

//int sp_fd = open_serial_port(port_name.c_str(), baud_rate, 8, 'N', 1);
int open_serial_port(const char *port_name, int baud_rate, int bits, char event, int stop)
{
    // open port
    int fd = open(port_name, O_RDWR | O_NOCTTY);

    if (-1 == fd)
    {
        perror("Serial Error: Failed to open serial port!\n");
        exit(-1);
    }

    fcntl(fd, F_SETFL, 0);

    // set_opt
    struct termios newtio, oldtio;
    tcgetattr(fd, &oldtio);

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CRTSCTS;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag &= ~OPOST;

    switch (bits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        goto ERROR_EXIT;
    }

    switch (event)
    {
    case 'o':
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'e':
    case 'E':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        goto ERROR_EXIT;
    }

    switch (baud_rate)
    {
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 230400:
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    switch (stop)
    {
    case 1:
        newtio.c_cflag &= ~CSTOPB;
        break;
    case 2:
        newtio.c_cflag |= CSTOPB;
        break;
    default:
        newtio.c_cflag &= ~CSTOPB;
        break;
    }

    tcflush(fd, TCIFLUSH);

    if (0 != tcsetattr(fd, TCSANOW, &newtio))
    {
        perror("Serial Error: Set option error\n");
        goto ERROR_EXIT;
    }

    return fd;

ERROR_EXIT:
    close(fd);
    return -1;
}

int read_nbyte(int fd, char *buf_ptr, const int nbytes)
{
    return read(fd, buf_ptr, nbytes);
}

int read_until(int fd, char *buf_ptr, const int buf_len, const char *delim)
{
    if (fd <= 0 || NULL == buf_ptr)
    {
        return 0;
    }

    char c = '\0';
    size_t nbytes = 0;
    size_t match_idx = 0;
    const size_t delim_len = strlen(delim);
    memset(buf_ptr, '\0', sizeof(char) * buf_len);

    while (true)
    {
        if (read(fd, &c, 1) >= 1)
        {
            (delim[match_idx] == c) ? (match_idx += 1) : (match_idx = 0);
            buf_ptr[nbytes++] = c;
            if ((1 == (buf_len - nbytes)) || (match_idx == delim_len))
            {
                break;
            }
        }
    }

    return nbytes;
}

/* Write "n" bytes to a descriptor. */
// ssize_t writen(int fd, const void *vptr, size_t n)
// {
//     size_t nleft;
//     ssize_t nwritten;
//     const char *ptr;

//     ptr = vptr;
//     nleft = n;
//     while (nleft > 0)
//     {
//         if ((nwritten = write(fd, ptr, nleft)) <= 0)
//         {
//             if (nwritten < 0 && errno == EINTR)
//                 nwritten = 0; /* and call write() again */
//             else
//                 return (-1); /* error */
//         }

//         nleft -= nwritten;
//         ptr += nwritten;
//     }
//     return (n);
// }
// /* end writen */

// void Writen(int fd, void *ptr, size_t nbytes)
// {
//     if (writen(fd, ptr, nbytes) != nbytes)
//         err_sys("writen error");
// }

// // int main()
// // {
// //     char *p1 = "This is a c test code";
// //     volatile int len = 0;

// //     int fp = open("/home/test.txt", O_RDWR | O_CREAT);
// //     Writen(fp,p1,strlen(p1));
// //     return 0;   
// // }