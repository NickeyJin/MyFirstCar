/*
 * board.h —— 板级杂项（放各层都要用的零碎定义）
 *
 * PC13 板载状态灯：**低电平点亮**，约 3mA，只能点灯，不能驱动负载。
 * 灯的含义由 app.c 定：闪 N 下 = 第 N 段/第 N 步；常亮 = 就绪；常灭 = 手柄掉线。
 */
#ifndef BOARD_H
#define BOARD_H

#include "main.h"

#define LED_ON()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#endif /* BOARD_H */
