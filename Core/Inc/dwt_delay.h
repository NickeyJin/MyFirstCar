/* 
dwt_delay.h —— 微秒级延时
作用：提供 delay_us()。是系统自带的 DWT 周期计数器。
用法：先调一次 dwt_init()，之后可以调用 delay_us(n) 来精确定时。
 */
#ifndef DWT_DELAY_H
#define DWT_DELAY_H

#include <stdint.h>

void dwt_init(void);
void delay_us(uint32_t us);

#endif /* DWT_DELAY_H */