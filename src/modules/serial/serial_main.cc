#include <nuttx/config.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

extern "C"
{
    int serial_main(int argc, char **argv)
    {
        int fd;
        fd = open("/dev/ttyS1", O_RDWR);

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }

        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 1; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 921600
        cfsetispeed(&tty, B921600);
        cfsetospeed(&tty, B921600);

        // Save tty settings, also checking for error
        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }

        int cnt = 0;
        uint8_t read_buf[36];
        if (fd < 0)
        {
            fprintf(stderr, "ERROR: open failed: %d\n", errno);
            return -1;
        }
        while (cnt < 50)
        {
            printf("waiting for data %d \n", cnt);
            ssize_t n = read(fd, read_buf, sizeof(read_buf));
            if (n > 0)
            {
                for (size_t i = 0; i < n; i++)
                {
                    printf("%c", read_buf[i]);
                }
            }
            usleep(10000); // 10 ms
            cnt++;
            
        }
        close(fd);
        printf("closing serial port ...\n");
        usleep(10000);
        return 1;
    }
}