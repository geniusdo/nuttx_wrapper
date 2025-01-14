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

#include "dshot.hpp"

extern "C"
{
    int dshot_main(int argc, FAR char *argv[])
    {
        DShot::DShot<4> dshotDrv;
        dshotDrv.init();
        dshotDrv.register_motor_channel_map(2, 1, 4, 3);
        dshotDrv.start();
        sleep(3);
        dshotDrv.set_motor_throttle(0.2, 0.0, 0.0, 0.0);
        sleep(1);
        dshotDrv.set_motor_throttle(0.2, 0.2, 0.0, 0.0);
        sleep(1);
        dshotDrv.set_motor_throttle(0.2, 0.2, 0.2, 0.0);
        sleep(1);
        dshotDrv.set_motor_throttle(0.2, 0.2, 0.2, 0.2);
        sleep(3);
        dshotDrv.deinit();
        return 1;
    }
}