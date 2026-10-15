/*
motor.c —— 把一个 -1000~+1000 的速度变成某个定时器两个通道上的 PWM 占空比，从而让电机开动
    speed_to_ccr(): 把速度映射到 CCR
    motor_write(): 写入 CCR，正负速度对应 IN1/IN2 两路 PWM
    motor_set(): 做一些自检，防止速度越界 + 调整轮子前后方向，然后运行 motor_write()
    motor_stop_all(): 让四个轮子都滑行停车
    motor_brake_all(): 让四个轮子都刹车停车（IN1=IN2=1）
    motor_init(): 上电先让轮子老实待着，顺便把定时器 PWM 开起来
*/
#include "main.h"
#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t           ch_in1;
    uint32_t           ch_in2;
    uint8_t            invert;   /* 1 = 这个轮子的电机线接反了，软件帮它翻正 */
} MotorMap_t;

/* 下标 0 占位，让轮号 = 数组下标，看着顺眼也少一次减法 */
static const MotorMap_t motor_map[5] =
{
    { NULL,   0,               0,               0 },
    { &htim2, TIM_CHANNEL_1,   TIM_CHANNEL_2,   0 },   /* 1 左前 FL: PA0 / PA1  */
    { &htim2, TIM_CHANNEL_3,   TIM_CHANNEL_4,   0 },   /* 2 右前 FR: PA2 / PA3  */
    { &htim3, TIM_CHANNEL_3,   TIM_CHANNEL_4,   0 },   /* 3 左后 RL: PB0 / PB1  */
    { &htim1, TIM_CHANNEL_1,   TIM_CHANNEL_4,   1 },   /* 4 右后 RR: PA8 / PA11 */
};

/* 速度 → CCR。满量程 speed 对应 CCR = ARR+1（整周期全高，真 100%） */
static uint32_t speed_to_ccr(int16_t speed)
{
    uint32_t duty = (uint32_t)speed * (MOTOR_PWM_ARR + 1U) / MOTOR_SPEED_MAX;

    if (duty > MOTOR_PWM_ARR + 1U) duty = MOTOR_PWM_ARR + 1U;   /* 理论到不了，保险 */
    return duty;
}

static void motor_write(const MotorMap_t *m, int16_t speed)
{
    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1, speed_to_ccr(speed));
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2, speed_to_ccr((int16_t)(-speed)));
    }
    else
    {
        /* IN1 = IN2 = 0：高阻滑行，不硬刹车，车轮还能被推着转 */
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2, 0);
    }
}

void motor_init(void)
{
    /* TIM2：左前 + 右前，4 路全开 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /* TIM3：左后 2 路 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* TIM1 是高级定时器，HAL_TIM_PWM_Start() 里会顺手打开主输出 MOE，
       少这一步 PA8 / PA11 是不会有波形的 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    motor_stop_all();   /* 上电先让轮子老实待着 */
}

void motor_set(uint8_t wheel, int16_t speed)
{
    if (wheel < MOTOR_FL || wheel > MOTOR_RR) return;

    const MotorMap_t *m = &motor_map[wheel];

    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

    if (m->invert) speed = (int16_t)(-speed);

    motor_write(m, speed);
}

void motor_stop_all(void)
{
    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++) motor_set(w, 0);
}

void motor_brake_all(void)
{
    /* IN1 = IN2 = 1：两路都拉满，DRV8833 上下桥同时导通 = 刹车 */
    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++)
    {
        const MotorMap_t *m = &motor_map[w];
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in1, MOTOR_PWM_ARR + 1U);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_in2, MOTOR_PWM_ARR + 1U);
    }
}
