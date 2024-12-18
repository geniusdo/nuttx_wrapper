#include "bmi08_port.h"
#include "bmi08x.h"
#include <cstdio>
#include <cmath>
#include <sys/time.h>

static float acc_x, acc_y, acc_z;
static float gyro_x, gyro_y, gyro_z;

#define GRAVITY_EARTH (9.80665f)
#define DEG_TO_RAD (M_PI / 180.0f)

/*********************************************************************/
/*                        Global variables                           */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
struct bmi08_sensor_data bmi08_accel;

/*! @brief variable to hold the bmi08 gyro data */
struct bmi08_sensor_data bmi08_gyro;

/*! bmi08 accel int config */
struct bmi08_accel_int_channel_cfg accel_int_config;

/*! bmi08 gyro int config */
struct bmi08_gyro_int_channel_cfg gyro_int_config;

static int8_t init_bmi08(void)
{
    int8_t rslt;

    rslt = bmi08a_init(&bmi08dev);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_400_HZ;

        if (bmi08dev.variant == BMI085_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08dev.variant == BMI088_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;    /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08dev);

        rslt = bmi08xa_set_meas_conf(&bmi08dev);

        bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_47_ODR_400_HZ;
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_47_ODR_400_HZ;
        bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev);

        rslt = bmi08g_set_meas_conf(&bmi08dev);
    }

    return rslt;
}

static int8_t enable_bmi08_interrupt()
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg *)&accel_int_config, &bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);

        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi08dev);
    }

    return rslt;
}

static int8_t disable_bmi08_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg *)&accel_int_config, &bmi08dev);
    // bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

        /* Disable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
        // bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}

extern "C"
{
    int imu_main(int argc, char **argv)
    {

        int8_t rslt;

        uint32_t times_to_read = 0;
        float x = 0.0, y = 0.0, z = 0.0;
        uint8_t status = 0;

        struct timeval t;

        rslt = bmi08_interface_init(&bmi08dev, BMI08_SPI_INTF, BMI088_VARIANT);
        printf("bmi interface init mark %d \n", rslt);
        if (rslt == BMI08_OK)
        {
            rslt = init_bmi08();
            printf("init bmi088 %d \n", rslt);
            if (rslt < 0)
                return -1;

            rslt = enable_bmi08_interrupt();

            if (rslt == BMI08_OK)
            {
                for (;;)
                {
                    uint8_t last_acce_flag = 0;
                    uint8_t last_gyro_flag = 0;
                    /* Read accel data ready interrupt status */
                    rslt = bmi08a_get_data_int_status(&status, &bmi08dev);

                    if ((status & BMI08_ACCEL_DATA_READY_INT) &&  (last_acce_flag == 0))
                    {
                        gettimeofday(&t,0);
                        rslt = bmi08a_get_data(&bmi08_accel, &bmi08dev);

                        acc_x = static_cast<float>(bmi08_accel.x) / 1365.0f * GRAVITY_EARTH;
                        acc_y = static_cast<float>(bmi08_accel.y) / 1365.0f * GRAVITY_EARTH;
                        acc_z = static_cast<float>(bmi08_accel.z) / 1365.0f * GRAVITY_EARTH;

                        rslt = bmi08g_get_data(&bmi08_gyro, &bmi08dev);
                        gyro_x = static_cast<float>(bmi08_gyro.x) / 16.384f * DEG_TO_RAD;
                        gyro_y = static_cast<float>(bmi08_gyro.y) / 16.384f * DEG_TO_RAD;
                        gyro_z = static_cast<float>(bmi08_gyro.z) / 16.384f * DEG_TO_RAD;
                        
                        double current_time = static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_usec) / 1000000.0;

                        printf("current time %f \n acce %4.2f %4.2f %4.2f\n gyro %4.2f %4.2f %4.2f\n",current_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
                        times_to_read++;
                    }
                    last_acce_flag = status & BMI08_ACCEL_DATA_READY_INT;
                    // last_gyro_flag = status & BMI08_GYRO_DATA_READY_INT;

                    if (times_to_read > 2000)
                    {
                        break;
                    }
                }
                rslt = disable_bmi08_interrupt();
            }

            return 1;
        }
    }
}