#include "dwt_delay.h"

void DWT_Delay_Init(void)
{
    // 使能DWT外设的跟踪功能
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 复位并使能DWT的周期计数器CYCCNT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t start_tick = DWT->CYCCNT;
    // 根据您的168MHz时钟，计算每微秒的tick数
    uint32_t ticks_per_us = (HAL_RCC_GetHCLKFreq() / 1000000);
    uint32_t delay_ticks = us * ticks_per_us;

    while (DWT->CYCCNT - start_tick < delay_ticks);
}

void DWT_Delay_ms(uint32_t ms)
{
    while(ms--)
    {
        DWT_Delay_us(1000);
    }
}
