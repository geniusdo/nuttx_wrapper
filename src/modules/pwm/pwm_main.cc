#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/timers/pwm.h>

extern "C"
{
    int pwm_main(int argc, FAR char *argv[])
    {
        uint32_t duty = 0;
        for (int i = 1; i < argc; ++i)
        {
            if (strcmp(argv[i], "-d") == 0 && i + 1 < argc)
            {
                duty = static_cast<uint32_t>(atoi(argv[i + 1]));
                i++;
            }
        }

        struct pwm_info_s info;
        int fd;
        int ret;
        fd = open("/dev/pwm0", O_RDONLY);
        if (fd < 0)
        {
            printf("pwm_main: open %s failed: %d\n", "/dev/pwm0", errno);
            return 1;
        }

        info.frequency = 50; // 50 Hz
        for (size_t i = 0; i < 4; i++)
        {
            info.channels[i].channel = i;
            info.channels[i].duty = static_cast<uint32_t>(65535.0f * 2.0f / 50.0f);
            printf(" channel: %d duty: %d \n",
                   i, info.channels[i].duty);
        }

        ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
                    (unsigned long)((uintptr_t)&info));
        if (ret < 0)
        {
            printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
                   errno);
            return 1;
        }

        ret = ioctl(fd, PWMIOC_START, 0);
        if (ret < 0)
        {
            printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
            return 1;
        }

        usleep(4000000); // 3s

        // min throttle
        for (size_t i = 0; i < 4; i++)
        {
            info.channels[i].channel = i;
            info.channels[i].duty = static_cast<uint32_t>(65535.0f * 1.0f / 50.0f);
            printf(" channel: %d duty: %d \n",
                   i, info.channels[i].duty);
        }
        ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
                    (unsigned long)((uintptr_t)&info));
        if (ret < 0)
        {
            printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
                   errno);
            return 1;
        }
        usleep(3000000); // 3s

        // min throttle
        for (size_t i = 0; i < 4; i++)
        {
            info.channels[i].channel = i;
            info.channels[i].duty = static_cast<uint32_t>(65535.0f * 1.0f / 50.0f);
            printf(" channel: %d duty: %d \n",
                   i, info.channels[i].duty);
        }
        ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
                    (unsigned long)((uintptr_t)&info));
        if (ret < 0)
        {
            printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
                   errno);
            return 1;
        }
        usleep(3000000); // 3s

        printf("pwm_main: stopping output\n");

        ret = ioctl(fd, PWMIOC_STOP, 0);
        if (ret < 0)
        {
            printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
            return 1;
        }
        close(fd);
        return 1;
    }
}