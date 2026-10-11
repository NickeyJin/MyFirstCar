/*
servo.c —— 舵机控制模块
    servo_set() 设置指定舵机的微秒脉宽
    servo_start() 启动所有舵机的 PWM 输出并初始化到最小位置
*/
#include "main.h"
#include "servo.h"

extern TIM_HandleTypeDef htim4;

void servo_set(uint8_t no, uint16_t us)
{
    if( us < SERVO_MIN ) us = SERVO_MIN;
    if( us > SERVO_MAX ) us = SERVO_MAX;

    switch(no)
    {
        case 1: htim4.Instance->CCR1 = us; break;
        case 2: htim4.Instance->CCR2 = us; break;
        case 3: htim4.Instance->CCR3 = us; break;
        case 4: htim4.Instance->CCR4 = us; break; 
        default: break;
        /*瞄了一眼源代码，想了想，写指针看起来最好看*/
    }
}

void servo_start(void)
{
    /* 启动所有舵机的 PWM 输出 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   /* PB6 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   /* PB7 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);   /* PB8 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);   /* PB9 */

    /* 初始化所有舵机到最小位置 */
    servo_set(1, SERVO_MIN);
    servo_set(2, SERVO_MIN);
    servo_set(3, SERVO_MIN);
    servo_set(4, SERVO_MIN);
}