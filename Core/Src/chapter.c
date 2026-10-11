#include "main.h"
#include "dwt_delay.h"
#include "ps2.h"
#include "servo.h"
#include "motor.h"
#include "mailun.h"

#define MAX_SPEED   600        /* 摇杆推到底 = 600 < 1000 */
#define DEADZONE    12         /* 摇杆不一定完美回到中位，留一点余量，给操作手容错空间 */
#define SWEEP_STEP  15         /* 舵机每 20ms 走多少 µs（单程 ≈2.7 秒） */
#define IDLE_MS     5000       /* 5 秒不按 START 就回 0 位 */


/*===================================测试用====================================*/
static void blink_n(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   /* 亮（低电平点亮） */
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);     /* 灭 */
        HAL_Delay(150);
    }
    HAL_Delay(700);                     /* 两组之间留个空档，好数 */
}

/* ========== 自检：摇杆 → 运动学 → 电机映射，不需要任何外部硬件 ========== */
static uint8_t motor_selftest(void)
{
    static const int16_t in[5][3] = {
        { 600,   0,   0 },      /* ① 前进 */
        {   0, 600,   0 },      /* ② 右移 */
        {   0,   0, 600 },      /* ③ 自转 */
        { 600, 600,   0 },      /* ④ 斜推：会触发归一化 */
        {   0,   0,   0 },      /* ⑤ 停 */
    };
    /* 期望每个轮子的符号：+1 正转、-1 反转、0 停 */
    static const int8_t exp[5][4] = {
        { +1, +1, +1, +1 },     /* 前进：四轮同向 */
        { -1, +1, +1, -1 },     /* 右移：对角线同向、两组反向 */
        { -1, +1, -1, +1 },     /* 自转：左倒右正 */
        {  0, +1, +1,  0 },     /* 斜推：归一化后 FR/RL 满速、FL/RR 为零 */
        {  0,  0,  0,  0 },     /* 停 */
    };

    for (uint8_t k = 0; k < 5; k++)
    {
        mecanum_drive(in[k][0], in[k][1], in[k][2]);

        for (uint8_t n = 1; n <= 4; n++)
        {
            uint32_t c1, c2;
            switch (n)
            {
                case 1:  c1 = TIM2->CCR1; c2 = TIM2->CCR2; break;   /* 左前 */
                case 2:  c1 = TIM2->CCR3; c2 = TIM2->CCR4; break;   /* 右前 */
                case 3:  c1 = TIM3->CCR3; c2 = TIM3->CCR4; break;   /* 左后 */
                default: c1 = TIM1->CCR1; c2 = TIM1->CCR4; break;   /* 右后 */
            }

            int8_t got = 0;
            if      (c1 > 0 && c2 == 0) got = +1;   /* IN1 出力、IN2 为零 → 正转 */
            else if (c1 == 0 && c2 > 0) got = -1;   /* 反过来 → 反转 */
            else if (c1 == 0 && c2 == 0) got = 0;   /* 两路都零 → 高阻滑行 */

            if (got != exp[k][n - 1])
            {
                return (uint8_t)(k * 4 + n);        /* 第几项错了：1~20 */
            }
        }
    }

    mecanum_drive(0, 0, 0);                         /* 收尾全停 */
    return 0;                                       /* 0 = 20 项全过 */
}
/*===================================测试用====================================*/

/* 将摇杆的原始值放缩到正移和反移的最大速度上 */
static int16_t stick(uint8_t raw)
{
    int16_t d = (int16_t)raw - 128;

    if (d > -DEADZONE && d < DEADZONE) return 0;

    if (d > 0) d -= DEADZONE;
    else       d += DEADZONE;

    return (int16_t)((int32_t)d * MAX_SPEED / (128 - DEADZONE));
}

/* 4 路舵机一起写同一角度 */
static void servo_all(uint16_t us)
{
    for (uint8_t i = 1; i <= 4; i++) servo_set(i, us);
}


// void chapter_main(void)
// {
//     dwt_init();
//     ps2_init();
//     motor_init();
//     servo_start();

//     HAL_Delay(500);
//     /*等电稳定*/

//     uint16_t servo_us    = SERVO_MIN;
//     int8_t   sweep_dir   = +1;
//     uint32_t last_active = HAL_GetTick();
//     uint32_t last = HAL_GetTick();

//     while (1)
//     {
//         ps2_poll();

//         if (ps2_ok)
//         {
//             /* 左摇杆 → 平动（前后/左右）；右摇杆左右 → 自转 */
//             int16_t vx = stick(ps2_ly);
//             int16_t vy = stick(ps2_lx);
//             int16_t w  = stick(ps2_rx);

//             /* 按住 START：底盘停住，专心看舵机（免得一边扫舵机一边车乱跑） */
//             if (ps2_btn1 & PS2_BTN_START)
//             {
//                 vx = 0; vy = 0; w = 0;

//                 last_active = HAL_GetTick();
//                 servo_us += sweep_dir * SWEEP_STEP;
//                 if (servo_us >= SERVO_MAX) { servo_us = SERVO_MAX; sweep_dir = -1; }
//                 if (servo_us <= SERVO_MIN) { servo_us = SERVO_MIN; sweep_dir = +1; }
//                 servo_all(servo_us);
//             }

//             mecanum_drive(vx, vy, w);
//         }
//         else
//         {
//             mecanum_drive(0, 0, 0);      /* 掉线/跑远 → 全车停车（高阻滑行） */
//         }

//         /* 5 秒没碰过 START，舵机自动回 0 位 */
//         if (HAL_GetTick() - last_active > IDLE_MS)
//         {
//             servo_us  = SERVO_MIN;
//             sweep_dir = +1;
//             servo_all(SERVO_MIN);
//         }

//         while( HAL_GetTick() - last < 20 ) {}  /* 保证 50Hz 主循环 */
//         last = HAL_GetTick();
//     }
// }


void chapter_main(void)
{
    dwt_init();
    motor_init();

    while (1)
    {
        motor_set(1,  1000);  HAL_Delay(1500);   /* 正转 1.5 秒 */
        motor_set(1,    0);  HAL_Delay(500);    /* 高阻滑行 0.5 秒 */
        motor_set(1, -1000);  HAL_Delay(1500);   /* 反转 1.5 秒 */
        motor_set(1,    0);  HAL_Delay(1500);   /* 停 1.5 秒 */
    }
}