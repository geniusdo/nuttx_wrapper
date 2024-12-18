/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    common.c
 * @brief   Common file for BMI08 examples
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "bmi08_port.h"

/*! BMI08 shuttle id */
#define BMI08_SHUTTLE_ID_1  UINT16_C(0x86)
#define BMI08_SHUTTLE_ID_2  UINT16_C(0x66)

uint8_t acc_dev_add;
uint8_t gyro_dev_add;

struct spi_dev_s *spi;

 // we dont port i2c interface currently
/*!
 * I2C read function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    // return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    // return coines_write_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * SPI read function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;


    (void)intf_ptr;
    SPI_LOCK(spi, true);
    SPI_SELECT(spi, device_addr, true);
    // SPI_SELECT(spi, 1, false);
    // SPI_SELECT(spi, 0, true);

    SPI_SEND(spi, reg_addr);

    for(size_t i = 0; i<len; i++)
    {
        *(reg_data + i) = SPI_SEND(spi, 0xff);
        // printf("data got %d \n", *(reg_data + i));
    }
    SPI_SELECT(spi, device_addr, false);
    SPI_LOCK(spi, false);


    return BMI08_INTF_RET_SUCCESS;
}

/*!
 * SPI write function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;
    SPI_LOCK(spi, true);
    SPI_SELECT(spi, device_addr, true);
    // return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
    SPI_SEND(spi, reg_addr);

    for(size_t i = 0; i<len; i++)
    {
        SPI_SEND(spi, *(reg_data + i));
        // printf("data write %d \n", *(reg_data + i));
    }
    SPI_SELECT(spi, device_addr, false);
    SPI_LOCK(spi, false);
    return BMI08_INTF_RET_SUCCESS;
}

/*!
 * Delay function map to COINES platform
 */
void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    // period = period << 1;
    usleep(period);
}

void bmi08_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI08_OK:

            /* Do nothing */
            break;
        case BMI08_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMI08_E_COM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMI08_E_INVALID_CONFIG:
            printf("API [%s] Error [%d] : Invalid configuration\r\n", api_name, rslt);
            break;
        case BMI08_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMI08_E_OUT_OF_RANGE:
            printf("API [%s] Error [%d] : Out of Range\r\n", api_name, rslt);
            break;
        case BMI08_E_INVALID_INPUT:
            printf("API [%s] Error [%d] : Invalid Input\r\n", api_name, rslt);
            break;
        case BMI08_E_CONFIG_STREAM_ERROR:
            printf("API [%s] Error [%d] : Config Stream error\r\n", api_name, rslt);
            break;
        case BMI08_E_RD_WR_LENGTH_INVALID:
            printf("API [%s] Error [%d] : Invalid Read-write length\r\n", api_name, rslt);
            break;
        case BMI08_E_FEATURE_NOT_SUPPORTED:
            printf("API [%s] Error [%d] : Feature not supported\r\n", api_name, rslt);
            break;
        case BMI08_W_FIFO_EMPTY:
            printf("API [%s] Warning [%d] : FIFO empty\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

void enable_accel()
{
    SPI_LOCK(spi, true);
    SPI_SELECT(spi, 1, false);
    SPI_SELECT(spi, 0, true);
    SPI_LOCK(spi, false);
    printf("switch to acc\n");

}

void enable_gyro()
{
    SPI_LOCK(spi, true);
    SPI_SELECT(spi, 0, false);
    SPI_SELECT(spi, 1, true);
    SPI_LOCK(spi, false);
    printf("switch to gyro\n");

}

int8_t bmi08_interface_init(struct bmi08_dev *bmi08, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMI08_OK;
    // struct coines_board_info board_info;

    spi = stm32_spibus_initialize(1);

    if (bmi08 != NULL)
    {
        SPI_LOCK(spi, true);
        SPI_SETMODE(spi, SPIDEV_MODE0);
        SPI_SETBITS(spi, 8);
        SPI_HWFEATURES(spi, 0);
        SPI_SETFREQUENCY(spi, 5000000);
        SPI_LOCK(spi, false);

        usleep(5000);

        /* Bus configuration : I2C */
        if (intf == BMI08_I2C_INTF)
        {

        }
        /* Bus configuration : SPI */
        else if (intf == BMI08_SPI_INTF)
        {
            printf("SPI Interface \n");

            bmi08->write = bmi08_spi_write;
            bmi08->read = bmi08_spi_read;

            bmi08->intf = BMI08_SPI_INTF;
        }
        acc_dev_add = 0;
        gyro_dev_add = 1; 
        bmi08->intf_ptr_accel = &acc_dev_add;
        bmi08->intf_ptr_gyro =  &gyro_dev_add;
        bmi08->delay_us = bmi08_delay_us;
        bmi08->read_write_len = 8;
        bmi08->variant = (enum bmi08_variant)variant;
        usleep(10000);
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

