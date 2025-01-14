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
#include <termios.h>
#include <mqueue.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>
#include <nuttx/board.h>
// #include "stm32_gpio.h"

#include "dshot_lower_drv.hpp"

extern "C"
{
    int dshot_main(int argc, FAR char *argv[])
    {
        uint32_t throttle = 0;
        for (int i = 1; i < argc; ++i)
        {
            if (strcmp(argv[i], "-d") == 0 && i + 1 < argc)
            {
                throttle = static_cast<uint32_t>(atoi(argv[i + 1]));
                i++;
            }
        }
        float throttle_f = float(throttle) / 100.0f;
        printf("throttle %f\n", throttle_f);
        stm32_dshot_timer_init();
        stm32_dshot_dma_init();

        set_channel_throttle(0, 0);
        set_channel_throttle(1, 0);
        set_channel_throttle(2, 0);
        set_channel_throttle(3, 0);

        dshot_all_channel_start();

        sleep(3);

        set_channel_throttle(0, 500);
        set_channel_throttle(1, 500);
        set_channel_throttle(2, 500);
        set_channel_throttle(3, 500);

        sleep(3);

        set_channel_throttle(0, 0);
        set_channel_throttle(1, 0);
        set_channel_throttle(2, 0);
        set_channel_throttle(3, 0);

        sleep(1);

        dshot_all_channel_stop();
        stm32_dshot_dma_deinit();
        stm32_dshot_timer_deinit();
        return 1;
    }
}