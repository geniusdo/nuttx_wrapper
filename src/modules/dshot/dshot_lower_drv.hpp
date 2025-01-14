#pragma once
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <stm32_tim.h>

#include <stm32_dma.h>

#include <stm32_gpio.h>

#define DSHOT_HIGH_BIT 14
#define DSHOT_LOW_BIT 7

#define DSHOT_CHANNEL_NUM 4
#define DSHOT_DATA_LENGTH 20

static uint32_t data_buffer[DSHOT_DATA_LENGTH * DSHOT_CHANNEL_NUM] = {0};
static stm32_tim_dev_s *tim_dev;
static DMA_HANDLE dma_handles[DSHOT_CHANNEL_NUM];
static stm32_dma_config_s dma_configs[DSHOT_CHANNEL_NUM];

const unsigned int DMA_CHANNEL_MAPS[DSHOT_CHANNEL_NUM] = {
    DMAMAP_DMA12_TIM1CH1_0,
    DMAMAP_DMA12_TIM1CH2_0,
    DMAMAP_DMA12_TIM1CH3_0,
    DMAMAP_DMA12_TIM1CH4_0,
};

const uint32_t TIM_CCR_ADDRESS[DSHOT_CHANNEL_NUM] = {
    STM32_TIM1_BASE + STM32_GTIM_CCR1_OFFSET,
    STM32_TIM1_BASE + STM32_GTIM_CCR2_OFFSET,
    STM32_TIM1_BASE + STM32_GTIM_CCR3_OFFSET,
    STM32_TIM1_BASE + STM32_GTIM_CCR4_OFFSET,
};

// channel index start from 0
void set_channel_throttle(const uint8_t channel, const uint16_t throttle)
{
    uint16_t packet = 0;
    uint16_t throttle_int = throttle;
    throttle_int <<= 1; // skip telemetry bit
    uint16_t csum_data = throttle_int;
    uint16_t csum = (csum_data ^ (csum_data >> 4) ^ (csum_data >> 8)) & 0xf;
    packet = (throttle_int << 4) | csum;
    for (uint8_t i = 1; i < 17; i++)
    {
        data_buffer[i + channel * DSHOT_DATA_LENGTH] = (packet & 0x8000) ? DSHOT_HIGH_BIT : DSHOT_LOW_BIT;
        packet <<= 1;
    }
    return;
}
// currently assume only use TIM1
void stm32_dshot_timer_init(void)
{
    tim_dev = stm32_tim_init(1);
    STM32_TIM_SETCLOCK(tim_dev, 10000000); // 10MHZ  0,1 micro seconds
    STM32_TIM_SETMODE(tim_dev, STM32_TIM_MODE_UP);
    STM32_TIM_SETCOUNTER(tim_dev, 0); // initial value for counter
    STM32_TIM_SETPERIOD(tim_dev, 16); // 17 microseconds interval
    STM32_TIM_ENABLEINT(tim_dev, ATIM_DIER_UDE |
                                     ATIM_DIER_CC1DE |
                                     ATIM_DIER_CC2DE |
                                     ATIM_DIER_CC3DE |
                                     ATIM_DIER_CC4DE); // enable dma update interrupt

    for (size_t i = 0; i < DSHOT_CHANNEL_NUM; i++)
    {
        STM32_TIM_SETCOMPARE(tim_dev, i + 1, 0);
        STM32_TIM_SETCHANNEL(tim_dev, i + 1, STM32_TIM_CH_OUTPWM);
    }
    return;
}

void stm32_dshot_timer_deinit(void)
{
    for (size_t i = 0; i < DSHOT_CHANNEL_NUM; i++)
    {
        STM32_TIM_SETCOMPARE(tim_dev, i + 1, 0);
    }
    usleep(10);
    stm32_tim_deinit(tim_dev);
    return;
}

void stm32_dshot_dma_init(void)
{
    for (size_t i = 0; i < DSHOT_CHANNEL_NUM; i++)
    {
        dma_handles[i] = stm32_dmachannel(DMA_CHANNEL_MAPS[i]);
        dma_configs[i].paddr = (uint32_t)TIM_CCR_ADDRESS[i]; // gpio
        dma_configs[i].maddr = (uint32_t)(&data_buffer[i * DSHOT_DATA_LENGTH]);
        dma_configs[i].ndata = DSHOT_DATA_LENGTH;
        dma_configs[i].cfg1 = DMA_SCR_MINC |
                              DMA_SCR_DIR_M2P |
                              DMA_SCR_PSIZE_32BITS |
                              DMA_SCR_MSIZE_32BITS |
                              DMA_SCR_CIRC;
        dma_configs[i].cfg2 = 0;
        stm32_dmasetup(dma_handles[i], &dma_configs[i]);
    }
    return;
}

void stm32_dshot_dma_deinit(void)
{
    for (size_t i = 0; i < DSHOT_CHANNEL_NUM; i++)
    {
        stm32_dmastop(dma_handles[i]);
        stm32_dmafree(dma_handles[i]);
    }
    return;
}

void dshot_all_channel_start(void)
{
    for (size_t i = 0; i < DSHOT_CHANNEL_NUM; i++)
    {
        stm32_dmasetup(dma_handles[i], &dma_configs[i]);
        stm32_dmastart(dma_handles[i], NULL, NULL, false);
    }

    STM32_TIM_ENABLE(tim_dev);
}

void dshot_all_channel_stop(void)
{
    for (size_t i = 0; i < DSHOT_CHANNEL_NUM; i++)
    {
        stm32_dmastop(dma_handles[i]);
    }
    STM32_TIM_DISABLE(tim_dev);
}
