#include <nuttx/config.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <mqueue.h>

#include "static_queue.hpp"

struct remoteCmd
{
    float throttle;
    float q_w;
    float q_x;
    float q_y;
    float q_z;
};

int serial_task(int argc, char *argv[])
{
    int fd;
    fd = open("/dev/ttyS1", O_RDWR);

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
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
        return -1;
    }
    struct mq_attr attr;

    attr.mq_maxmsg = 20;
    attr.mq_msgsize = sizeof(remoteCmd);
    attr.mq_flags = 0;
    mqd_t g_send_mqfd;
    g_send_mqfd = mq_open("mqueue", O_WRONLY | O_CREAT, 0666, &attr);
    if (g_send_mqfd == (mqd_t)-1)
    {
        printf("sender_thread: ERROR mq_open failed, errno=%d\n", errno);
        return -1;
    }

    printf("creating circular buffer \n");
    StaticQueue<uint8_t, 100> msgBuffer{};
    remoteCmd cmd;
    int cnt = 0;
    uint8_t read_buf[25];
    uint8_t msg[25];
    if (fd < 0)
    {
        fprintf(stderr, "ERROR: open failed: %d\n", errno);
        return -1;
    }
    int byte_cnt = 0;
    while (1)
    {
        ssize_t read_count = read(fd, read_buf, 25);
        if (read_count > 0)
        {
            for (size_t i = 0; i < read_count; i++)
            {
                msgBuffer.push(read_buf[i]);
            }
            while (msgBuffer.front() != 0x0F && !msgBuffer.empty())
            {
                msgBuffer.pop();
            }
            if (msgBuffer.size() >= 25)
            {
                // printf("front data %02x \n", msgBuffer.front());

                for (size_t i = 0; i < 25; i++)
                {
                    msg[i] = msgBuffer.front();
                    msgBuffer.pop();
                }

                if (msg[24] == 0xEE)
                {
                    uint32_t tmp = msg[4] << 24 | msg[3] << 16 | msg[2] << 8 | msg[1];
                    memcpy(&(cmd.throttle), &tmp, 4);
                    mq_send(g_send_mqfd, reinterpret_cast<const char *>(&cmd), sizeof(remoteCmd), 42);
                }
            }
        }

        usleep(10000); // 10 ms
        cnt++;
    }
    mq_close(g_send_mqfd);
    close(fd);
    printf("closing serial port ...\n");
    usleep(10000);
    return 1;
}

int receive_task(int argc, char **argv)
{
    mqd_t g_recv_mqfd;
    struct mq_attr attr;
    attr.mq_maxmsg = 20;
    attr.mq_msgsize = sizeof(remoteCmd);
    attr.mq_flags = 0;
    g_recv_mqfd = mq_open("mqueue", O_RDONLY | O_CREAT, 0666, &attr);

    remoteCmd recCmd;
    int cnt = 0;
    while (cnt < 500)
    {
        ssize_t nbytes = mq_receive(g_recv_mqfd, reinterpret_cast<char *>(&recCmd), sizeof(remoteCmd), 0);
        printf("got %d bytes\n", nbytes);
        if (nbytes > 0)
        {
            printf("get %d msgs throttle %f \n", cnt, recCmd.throttle);
        }
        cnt++;
        usleep(10000); // 10 ms
    }

    mq_close(g_recv_mqfd);
    return 1;
}

extern "C"
{
    int serial_main(int argc, char **argv)
    {
        task_create("remote_task", 50, 16384, serial_task, NULL);
        // task_create("receive_task", 42, 4096, receive_task, NULL);
        // serial_task();
        return 1;
    }
}