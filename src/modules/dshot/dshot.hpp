#pragma once

#include "dshot_lower_drv.hpp"

namespace DShot
{

    template <int channels>
    class DShot
    {
    public:
        DShot() = default;
        ~DShot() { deinit(); }

        /// @brief initialize the dshot driver
        void init()
        {
            if (init_flag == 0)
            {
                stm32_dshot_timer_init();
                stm32_dshot_dma_init();
                init_flag = 1;
            }
        }

        /// @brief deinitialize the dshot driver
        void deinit()
        {
            if (init_flag == 1)
            {
                dshot_all_channel_stop();
                stm32_dshot_timer_deinit();
                stm32_dshot_dma_deinit();
                init_flag = 0;
            }
        }

        /// @brief start all dshot channel signals
        /// @return
        bool start()
        {
            if (init_flag == 1)
            {
                set_motor_throttle(0.0f, 0.0f, 0.0f, 0.0f);
                dshot_all_channel_start();
                
                return true;
            }
            return false;
        }

        /// @brief stop all dshot channel signals
        /// @return
        bool stop()
        {
            if (init_flag == 1)
            {
                dshot_all_channel_stop();
                return true;
            }
            return false;
        }

        /// @brief register the corresponding dshot channel of each motor
        /// @param motor1 the corresponding dshot channel of motor1
        /// @param motor2 the corresponding dshot channel of motor2
        /// @param motor3 the corresponding dshot channel of motor3
        /// @param motor4 the corresponding dshot channel of motor4
        /// @return true if input values are valid
        bool register_motor_channel_map(const uint8_t motor1, const uint8_t motor2, const uint8_t motor3, const uint8_t motor4)
        {
            if (motor1 + motor2 + motor3 + motor4 != (1 + 2 + 3 + 4))
                return false;
            motor_channel_map[0] = motor1 - 1;
            motor_channel_map[1] = motor2 - 1;
            motor_channel_map[2] = motor3 - 1;
            motor_channel_map[3] = motor4 - 1;
            motor_channel_mapped = 1;
            return true;
        }

        /// @brief set the throttle of each motor
        /// @param motor1 motor1 throttle from [0.0f,1.0f]
        /// @param motor2
        /// @param motor3
        /// @param motor4
        void set_motor_throttle(const float motor1, const float motor2, const float motor3, const float motor4)
        {
            if (motor_channel_mapped == 0)
                return;
            uint16_t motor1_throttle = motor1 > 0.001f ? uint16_t(motor1 * 2000.0f) + 48 : uint16_t(0);
            uint16_t motor2_throttle = motor2 > 0.001f ? uint16_t(motor2 * 2000.0f) + 48 : uint16_t(0);
            uint16_t motor3_throttle = motor3 > 0.001f ? uint16_t(motor3 * 2000.0f) + 48 : uint16_t(0);
            uint16_t motor4_throttle = motor4 > 0.001f ? uint16_t(motor4 * 2000.0f) + 48 : uint16_t(0);
            set_channel_throttle(motor_channel_map[0], motor1_throttle);
            set_channel_throttle(motor_channel_map[1], motor2_throttle);
            set_channel_throttle(motor_channel_map[2], motor3_throttle);
            set_channel_throttle(motor_channel_map[3], motor4_throttle);
            return;
        }

    private:
        uint8_t init_flag = 0;
        uint8_t motor_channel_mapped = 0;
        uint8_t motor_channel_map[channels];
    };
}