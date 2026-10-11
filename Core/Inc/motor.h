/* 
motor.h —— DRV8833 双 PWM 电机驱动
作用：控制四个电机的转速和方向。
用法：先motor_init(); 然后 motor_set(轮号, -1000~+1000) 从而控制电机。
*/
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_SPEED_MAX  1000      /* 速度满量程 */
#define MOTOR_PWM_FULL    3599      /* = TIM1/2/3 的 ARR */

/* 轮号：1=左前FL  2=右前FR  3=左后RL  4=右后RR */
void motor_init(void);
void motor_set(uint8_t no, int16_t speed);

#endif