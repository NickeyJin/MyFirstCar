/*
servo.h —— 4个舵机
作用：操控舵机
硬件：PB6~9 对应 TIM4_CH1~4
用法：servo_start();  然后 servo_set(1~4, 500~2500);
*/
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_MIN   500      /* 0° 端（脉宽 us）*/
#define SERVO_MAX  2500      /* 180° 端 */

void servo_start(void);                     /* 启动 4 路 PWM，先全部摆到 0° */
void servo_set(uint8_t no, uint16_t us);    /* no = 1~4 */

#endif /* SERVO_H */