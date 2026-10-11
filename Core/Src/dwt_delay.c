/*
dwt_delay.c —— 基于 DWT 的微秒级延时实现
    dwt_init() 初始化 DWT
    delay_us() 延时指定微秒数
*/
#include "stm32f1xx_hal.h"
#include "dwt_delay.h"

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}