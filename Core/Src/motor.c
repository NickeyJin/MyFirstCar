/*
motor.c —— DRV8833 双 PWM 电机驱动
    motor_init() 初始化电机
    motor_set(轮号, -1000~1000) 设置电机速度

DRV8833 真值表（一片管两个电机，一个电机占 IN1/IN2 两根）：
   IN1=0   IN2=0    高阻滑行
   IN1=PWM IN2=0    正转
   IN1=0   IN2=PWM  反转
   IN1=1   IN2=1    刹车

*/
#include "main.h"
#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t ch_in1;
    uint32_t ch_in2;
    uint8_t invert;      /* 1 = 这个轮子的电机线接反了，翻正一下 */
} MotorMap_t;

static const MotorMap_t motor_map[5] =
{
    { 0,0,0,0 },   /* 占位不用，最喜欢下标从1开始 */
    { &htim2, TIM_CHANNEL_2,  TIM_CHANNEL_3,  0 },   /* 1 左前 FL: PA0 / PA1 ，修改过，记得改回来*/  
    { &htim2, TIM_CHANNEL_3,  TIM_CHANNEL_4,  0 },   /* 2 右前 FR: PA2 / PA3 */
    { &htim3, TIM_CHANNEL_3,  TIM_CHANNEL_4,  0 },   /* 3 左后 RL: PB0 / PB1 */
    { &htim1, TIM_CHANNEL_1,  TIM_CHANNEL_4,  0 },   /* 4 右后 RR: PA8 / PA11 */
};

void motor_init(void)
{
    /* 8 路 PWM 全部启动启动启动，还有这个！ */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 轮子我求你先别动 */
    for (uint8_t i = 1; i <= 4; i++) motor_set(i, 0);
}

void motor_set(uint8_t no, int16_t speed)
{
    if (no < 1 || no > 4) return;

    const MotorMap_t *m = &motor_map[no];

    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

    if (m->invert) speed = -speed;

    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1,(uint32_t)speed * MOTOR_PWM_FULL / MOTOR_SPEED_MAX);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2,(uint32_t)(-speed) * MOTOR_PWM_FULL / MOTOR_SPEED_MAX);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2, 0);
    }
}