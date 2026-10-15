/*
 * diag.c —— 开发期排查代码（正常跑车时整份文件都不会被编译，开关在 diag.h）
 *
 * 这里所有函数都直接操作引脚和定时器寄存器，故意绕开正式的 motor / chassis 层，
 * 目的只有一个：把"轮子不转"拆成几段，一段一段量。
 * 判读方法、实测结果、结论表都写在 diag.h 里，这里只留必要的行内注释。
 */
#include "main.h"
#include "board.h"
#include "diag.h"
#include "dwt_delay.h"
#include "motor.h"

#if (APP_DIAG_MODE || APP_RR_TEST)
/* main.h 里没有引出定时器句柄，motor.c 也是自己 extern 的 */
extern TIM_HandleTypeDef htim1, htim2, htim3;

/* 每段开始前先闪 n 下灯 = "现在跑到第 n 段"，让排查的人不用看串口也能对上号 */
static void seg_cue(uint8_t n)
{
    LED_OFF();
    HAL_Delay(800);
    for (uint8_t i = 0; i < n; i++)
    {
        LED_ON();  HAL_Delay(200);
        LED_OFF(); HAL_Delay(250);
    }
    HAL_Delay(400);
}
#endif

#if APP_DIAG_MODE
/* ==================== 占空比 / 频率对比测试 ====================
 * 8 个 IN 脚按端口分组，方便一会儿整片写电平：
 *   PA: FL_IN1=PA0  FL_IN2=PA1  FR_IN1=PA2  FR_IN2=PA3  RR_IN1=PA8  RR_IN2=PA11
 *   PB: RL_IN1=PB0  RL_IN2=PB1
 */
#define DIAG_IN1_A      (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_8)
#define DIAG_IN2_A      (GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_11)
#define DIAG_ALL_A      (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_11)
#define DIAG_IN1_B      (GPIO_PIN_0)
#define DIAG_IN2_B      (GPIO_PIN_1)
#define DIAG_ALL_B      (GPIO_PIN_0|GPIO_PIN_1)

/* 把 8 个 IN 脚从"定时器复用"切成"普通推挽输出" */
static void diag_pins_to_gpio(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = DIAG_IN1_A | DIAG_IN2_A;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = DIAG_IN1_B | DIAG_IN2_B;
    HAL_GPIO_Init(GPIOB, &g);
}

/* 切回"定时器复用推挽"，把引脚还给 TIM1 / TIM2 / TIM3 */
static void diag_pins_to_af(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = DIAG_IN1_A | DIAG_IN2_A;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = DIAG_IN1_B | DIAG_IN2_B;
    HAL_GPIO_Init(GPIOB, &g);
}

static void diag_all_low(void)
{
    HAL_GPIO_WritePin(GPIOA, DIAG_ALL_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DIAG_ALL_B, GPIO_PIN_RESET);
}

/* 闪 2 用：GPIO 直推，手动翻出低频方波。
 * period_us = 周期，on_us = 高电平时间，两个一除就是占空比。 */
static void diag_gpio_pwm(uint32_t ms, uint32_t period_us, uint32_t on_us)
{
    uint32_t t0 = HAL_GetTick();

    diag_pins_to_gpio();

    HAL_GPIO_WritePin(GPIOA, DIAG_IN2_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DIAG_IN2_B, GPIO_PIN_RESET);

    while ((HAL_GetTick() - t0) < ms)
    {
        HAL_GPIO_WritePin(GPIOA, DIAG_IN1_A, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, DIAG_IN1_B, GPIO_PIN_SET);
        delay_us(on_us);

        HAL_GPIO_WritePin(GPIOA, DIAG_IN1_A, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, DIAG_IN1_B, GPIO_PIN_RESET);
        delay_us(period_us - on_us);
    }

    diag_all_low();
    motor_stop_all();
    diag_pins_to_af();
}

/* 闪 1 / 闪 3 用：走正常路径 —— 定时器 PWM + motor_set（speed 用 MOTOR_SPEED_MAX 刻度） */
static void diag_pwm(int16_t speed, uint32_t ms)
{
    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++) motor_set(w, speed);
    HAL_Delay(ms);
    motor_stop_all();
}

/* 闪 3 用：临时把三个定时器改成 1kHz，占空比 80%。
 * 和闪 1 的 20kHz/80% 只差频率，用来确认"降到 1kHz 会不会更快/更稳"。 */
static void diag_lowfreq_pwm(uint32_t ms)
{
    const uint32_t ccr = 800U;      /* 1000 个计数里占 800 = 80% */

    motor_stop_all();
    HAL_Delay(30);

    /* 72MHz / (71+1) = 1MHz 计数   →   1MHz / (999+1) = 1kHz PWM */
    __HAL_TIM_SET_PRESCALER(&htim1, 71U);
    __HAL_TIM_SET_PRESCALER(&htim2, 71U);
    __HAL_TIM_SET_PRESCALER(&htim3, 71U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 999U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 999U);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 999U);

    /* 绕开 motor_set（它按 MOTOR_PWM_ARR 换算），直接写 CCR：IN1 = 占空比，IN2 = 0 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    HAL_Delay(ms);

    motor_stop_all();
    HAL_Delay(30);

    /* 恢复原样：PSC=71 / ARR=MOTOR_PWM_ARR → 1kHz */
    __HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_PWM_ARR);
    __HAL_TIM_SET_AUTORELOAD(&htim2, MOTOR_PWM_ARR);
    __HAL_TIM_SET_AUTORELOAD(&htim3, MOTOR_PWM_ARR);
    __HAL_TIM_SET_PRESCALER(&htim1, 71U);
    __HAL_TIM_SET_PRESCALER(&htim2, 71U);
    __HAL_TIM_SET_PRESCALER(&htim3, 71U);
}

void diag_run(void)
{
    for (;;)
    {
        seg_cue(1);   /* 20kHz，占空比 80% */
        diag_pwm(800, 3000);
        HAL_Delay(2000);

        seg_cue(2);   /* GPIO 直推 100Hz，占空比 40% */
        diag_gpio_pwm(3000, 10000, 4000);
        HAL_Delay(2000);

        seg_cue(3);   /* 1kHz，占空比 80% */
        diag_lowfreq_pwm(3000);
        HAL_Delay(2500);
    }
}
#endif /* APP_DIAG_MODE */

#if APP_RR_TEST
/* ==================== 右后轮专项排查 ==================== */
#define RR_SEG_MS     3000      /* 每段驱动多久 */
#define RR_GAP_MS     1500      /* 段与段之间停多久 */

/* 把 PA8 / PA11 从"定时器复用"切成"普通推挽输出"，先都拉低 */
static void rr_pins_gpio(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = GPIO_PIN_8 | GPIO_PIN_11;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_11, GPIO_PIN_RESET);
}

/* 切回"定时器复用"，并把两路都写成 0% = 两路都低 = 高阻滑行 */
static void rr_pins_af(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = GPIO_PIN_8 | GPIO_PIN_11;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
}

/* in1 非 0 → PA8 拉高；in2 非 0 → PA11 拉高（闪 1 / 闪 2 用） */
static void rr_gpio(uint8_t in1, uint8_t in2)
{
    rr_pins_gpio();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* use_in2 非 0 → PA11 走 PWM（前进）；否则 PA8 走 PWM（后退）。
 * CCR > ARR = 恒高 = 100%。闪 3 / 闪 4 用。 */
static void rr_pwm(uint8_t use_in2)
{
    rr_pins_af();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, use_in2 ? 0U : (MOTOR_PWM_ARR + 1U));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, use_in2 ? (MOTOR_PWM_ARR + 1U) : 0U);
}

void diag_rr_run(void)
{
    for (;;)
    {
        seg_cue(1);  rr_gpio(0, 1);  HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
        seg_cue(2);  rr_gpio(1, 0);  HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
        seg_cue(3);  rr_pwm(1);      HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
        seg_cue(4);  rr_pwm(0);      HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
    }
}
#endif /* APP_RR_TEST */
