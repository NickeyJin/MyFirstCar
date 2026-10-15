/*
 * app.c —— 应用层（上电自检 + PS2 手柄全向控制）
 *
 * 职责：
 *   1. 上电自检：四个轮子挨个正转一下、反转一下，再四轮一起前后走一下，
 *      最后横移一步 —— 这样不用手柄也能一眼看出"哪个轮子没接 / 哪个轮子接反了 /
 *      横移方向对不对"。
 *   2. 主循环：读 PS2 → 左摇杆平移、右摇杆自转 → 过一遍加减速斜坡 →
 *      交给 chassis 层做麦轮逆运动学。
 *
 * 安全策略：只要手柄没数据（ps2_ok == 0），立刻四轮滑行停车；
 *           按住 START 当作急停按钮。
 */
#include "main.h"
#include "app.h"
#include "chassis.h"
#include "motor.h"
#include "ps2.h"
#include "teach_only.h"   /* 机械臂示教录制/回放 */
#include "dwt_delay.h"

/* 排查模式要临时改定时器，这里自己引一下（motor.c 里也是这么用的） */
#if APP_DIAG_MODE
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
#endif

/* 自检参数 */
#define TEST_SPEED       600    /* 自检转速：实测这颗电机 40% 才起步，自检用 60% 才看得清 */
#define TEST_RUN_MS      300    /* 每个动作持续多久 */
#define TEST_GAP_MS      150    /* 动作之间的停顿 */
#define TEST_WHEEL_GAP   250    /* 两个轮子之间的停顿 */

/* PC13 状态灯：低电平点亮 */
#define LED_ON()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

/* ==================== 电机排查（APP_DIAG_MODE） ====================
 * 上电后先闪灯报段号，再驱动电机。
 * 前面 A/B/C/D 四段已经测过（结果：GPIO 常高转、20kHz 100% 转、20kHz 40% 不转、
 * 100Hz 50% 转 —— 说明硬件链路和定时器都没问题，只有"20kHz + 小占空比"不行），
 * 现在只剩三段，用来区分到底是"占空比太低"还是"频率太高"：
 *   闪 1：定时器 PWM 20kHz，占空比 80%   —— 和当初不转的 C 段(20kHz/40%) 比，只改了占空比
 *   闪 2：GPIO 低频方波 100Hz，占空比 40% —— 和当初不转的 C 段(20kHz/40%) 比，只改了频率
 *   闪 3：定时器 PWM 1kHz，占空比 80%    —— 直接验证打算写进正式代码的最终配置
 * 判读：闪 1 转 → 占空比问题；闪 2 转 → 频率问题；闪 3 转 → 最终方案可用。
 *
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

static uint32_t app_last_tick;

/* ==================== 摇杆 → 速度 ====================
 * PS2 的轴值 0~255，中点是 0x80。分两步处理：
 *   1) stick_raw()：抠掉死区，输出 -115 ~ +115（保留符号和幅度）
 *   2) 再按 speed_max 归一化：摇杆幅值映射到 APP_SPEED_MIN ~ speed_max，
 *      而不是 0 ~ speed_max —— 实测这颗电机低占空比带不动，从 0 映射的话
 *      摇杆前一半行程全是"死"的。
 */
static int16_t stick_raw(uint8_t raw)
{
    int16_t d = (int16_t)raw - PS2_STICK_CENTER;

    if (d > -APP_STICK_DEADZONE && d < APP_STICK_DEADZONE) return 0;

    /* 把死区抠掉之后重新归一化，保证推到底正好是满幅 */
    if (d > 0) d = (int16_t)(d - APP_STICK_DEADZONE);
    else       d = (int16_t)(d + APP_STICK_DEADZONE);

    return d;
}

/* 单轴摇杆 → 一个速度值（自转轴用） */
static int16_t stick_to_speed(uint8_t raw, int16_t speed_max)
{
    int16_t d = stick_raw(raw);
    int32_t mag;
    const int32_t span = PS2_STICK_MAX - APP_STICK_DEADZONE;   /* 死区外还剩多少格 */

    if (d == 0) return 0;

    mag = (d < 0) ? -(int32_t)d : (int32_t)d;
    mag = APP_SPEED_MIN + mag * (speed_max - APP_SPEED_MIN) / span;
    if (mag > speed_max) mag = speed_max;

    return (int16_t)((d < 0) ? -mag : mag);
}

/* 左摇杆（两轴）→ 平移向量 vx / vy。
 * 幅值取两轴里较大的那个，再把两轴按同一比例缩放 —— 这样斜推就是斜着走，
 * 而不是"两个轴各自映射一遍"，那样斜推的方向会被扭曲。 */
static void stick_to_vector(uint8_t raw_x, uint8_t raw_y, int16_t speed_max,
                            int16_t *vx, int16_t *vy)
{
    int16_t jx =  stick_raw(raw_x);             /* > 0 = 向右推 */
    int16_t jy = (int16_t)(-stick_raw(raw_y));  /* > 0 = 向前推（PS2 上 = 0x00） */
    int16_t ax = (jx < 0) ? (int16_t)(-jx) : jx;
    int16_t ay = (jy < 0) ? (int16_t)(-jy) : jy;
    int16_t m  = (ax > ay) ? ax : ay;           /* 摇杆推离中心的幅度 */
    int32_t sp;
    const int32_t span = PS2_STICK_MAX - APP_STICK_DEADZONE;

    if (m == 0) { *vx = 0; *vy = 0; return; }

    sp = APP_SPEED_MIN + (int32_t)m * (speed_max - APP_SPEED_MIN) / span;
    if (sp > speed_max) sp = speed_max;

    *vx = (int16_t)((int32_t)jy * sp / m);
    *vy = (int16_t)((int32_t)jx * sp / m);
}

/* ==================== 加减速斜坡 ====================
 * 每拍（20ms）最多只让速度变化 APP_RAMP_STEP，这样从 0 升到 APP_SPEED_MAX
 * 大约要 APP_RAMP_MS 毫秒，起步不冲、麦轮不打滑、驱动板电流也温和。
 *
 * 唯一例外：从"停住"变成"要动"的那一下，直接跳到 APP_SPEED_MIN。
 * 因为 40% 以下根本带不动，慢慢爬过这一段只是白等；而且 chassis 层会在
 * 0→非 0 的那一拍"踹一脚"（满占空比 80ms）冲破静摩擦，两者配合刚好。 */
#define APP_RAMP_STEP   ((APP_SPEED_MAX * APP_LOOP_MS) / APP_RAMP_MS)

static int16_t ramp_to(int16_t cur, int16_t target)
{
    int32_t delta;

    if (cur == 0 && target != 0)
    {
        int32_t t     = (target < 0) ? -(int32_t)target : (int32_t)target;
        int32_t start = APP_SPEED_MIN;

        if (start > t) start = t;
        return (int16_t)((target > 0) ? start : -start);
    }

    delta = (int32_t)target - (int32_t)cur;
    if (delta >  APP_RAMP_STEP) delta =  APP_RAMP_STEP;
    if (delta < -APP_RAMP_STEP) delta = -APP_RAMP_STEP;

    return (int16_t)((int32_t)cur + delta);
}

/* 斜坡的当前值（三个自由度各一个） */
static int16_t ramp_vx, ramp_vy, ramp_w;

#if APP_SELFTEST_ENABLE
/* 单独点动一个轮子：先按"车前进"的方向转，再反过来 */
static void test_wheel(uint8_t wheel, const char *tag)
{
    (void)tag;
    chassis_wheel(wheel,  TEST_SPEED);  HAL_Delay(TEST_RUN_MS);
    chassis_wheel(wheel,  0);           HAL_Delay(TEST_GAP_MS);
    chassis_wheel(wheel, -TEST_SPEED);  HAL_Delay(TEST_RUN_MS);
    chassis_wheel(wheel,  0);           HAL_Delay(TEST_WHEEL_GAP);
}

static void self_test(void)
{
    /* 灯闪 3 下 = 我要开始动轮子了，手别放轮子上 */
    for (uint8_t i = 0; i < 3; i++)
    {
        LED_ON();  HAL_Delay(150);
        LED_OFF(); HAL_Delay(150);
    }
    HAL_Delay(300);

    test_wheel(MOTOR_FL, "FL");
    test_wheel(MOTOR_FR, "FR");
    test_wheel(MOTOR_RL, "RL");
    test_wheel(MOTOR_RR, "RR");

    /* 四轮一起：前进 → 后退 → 停 */
    chassis_forward(TEST_SPEED);   HAL_Delay(500);
    chassis_stop();                HAL_Delay(TEST_GAP_MS);
    chassis_backward(TEST_SPEED);  HAL_Delay(500);
    chassis_stop();                HAL_Delay(TEST_GAP_MS);

    /* 横移一步（向右）：验证麦轮的横移方向。
       如果看到车往左走，把 chassis.h 里的 CHASSIS_STRAFE_SIGN 改成 (-1) */
    chassis_strafe_right(TEST_SPEED); HAL_Delay(500);
    chassis_stop();                   HAL_Delay(TEST_GAP_MS);

    /* 灯常亮 = 自检结束，准备接手柄 */
    LED_ON();
}
#endif /* APP_SELFTEST_ENABLE */

#if APP_DIAG_MODE

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

/* 每段开始前先闪 n 下灯，让用户能数出"现在跑到第几段" */
static void diag_cue(uint8_t n)
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

/* D / F 段：GPIO 直推，手动翻出低频方波。
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

/* B / C / E 段：走正常路径 —— 定时器 PWM + motor_set */
static void diag_pwm(int16_t speed, uint32_t ms)
{
    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++) motor_set(w, speed);
    HAL_Delay(ms);
    motor_stop_all();
}

/* G 段：临时把三个定时器从 20kHz 降到 1kHz，占空比 80%。
 * 用来区分"C 段不转"到底是 20kHz 太快，还是 40% 太低：
 *   G 转 → 就是频率太高，正式代码把 PWM 降下来即可
 *   G 不转 → 那就是占空比不够，得提高最低速度 */
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

    /* 绕开 motor_set（它按 3600 计数换算），直接写 CCR：IN1 = 占空比，IN2 = 0 */
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

    /* 恢复原样：PSC=71 / ARR=999 → 1kHz（和 motor.h 里的 MOTOR_PWM_ARR 保持一致） */
    __HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_PWM_ARR);
    __HAL_TIM_SET_AUTORELOAD(&htim2, MOTOR_PWM_ARR);
    __HAL_TIM_SET_AUTORELOAD(&htim3, MOTOR_PWM_ARR);
    __HAL_TIM_SET_PRESCALER(&htim1, 71U);
    __HAL_TIM_SET_PRESCALER(&htim2, 71U);
    __HAL_TIM_SET_PRESCALER(&htim3, 71U);
}

static void diag_run(void)
{
    for (;;)
    {
        diag_cue(1);   /* 闪 1 下：20kHz，占空比 80% */
        diag_pwm(800, 3000);
        HAL_Delay(2000);

        diag_cue(2);   /* 闪 2 下：100Hz，占空比 40% */
        diag_gpio_pwm(3000, 10000, 4000);
        HAL_Delay(2000);

        diag_cue(3);   /* 闪 3 下：1kHz，占空比 80% */
        diag_lowfreq_pwm(3000);
        HAL_Delay(2500);
    }
}

#endif /* APP_DIAG_MODE */

#if APP_RR_TEST
/* ==================== 右后轮专项排查（APP_RR_TEST） ====================
 * 只动右后轮，四段各 3 秒，靠闪灯报段号（闪 N 下 = 第 N 段）：
 *
 *   闪 1 下 → PA11 当普通 GPIO 直接拉高（IN2=3.3V, IN1=0） 期望：前进方向
 *   闪 2 下 → PA8  当普通 GPIO 直接拉高（IN1=3.3V, IN2=0） 期望：后退方向
 *   闪 3 下 → PA11 走 TIM1_CH4 的 PWM（100%）              期望：前进方向
 *   闪 4 下 → PA8  走 TIM1_CH1 的 PWM（100%）              期望：后退方向
 *
 * 每段之间两路都拉低 = 高阻滑行（方便看清"到底哪一段没动"），然后停 1.5 秒。
 *
 * 2026-10-11 实测结果：
 *   闪 1、闪 3 不动，闪 2、闪 4 正常转。
 *   也就是说"GPIO 直推"和"定时器 PWM"两条路在 PA11 上全都不动，而在 PA8 上都好。
 *   ⇒ 跟定时器、跟软件无关，故障在 PA11 这一侧（或者是这个轮子的机械问题，见下）。
 *   曾把 RR 的 IN2 临时挪到 PA9 / TIM1_CH2 来绕开，但 **PCB 已经把 PA11 → DRV8833 IN2
 *   画死了，不能改线**，所以又改回 PA11。
 *
 * ！！这个测试**没法区分下面这两种情况**，它们看起来一模一样（都是"这一段轮子不动"）：
 *     ① PA11 真的没有输出（MCU 脚 / PCB 走线 / 焊点 / DRV8833 输入脚）；
 *     ② 脚有输出，但这个轮子在"前进"这个机械方向上转不动（减速箱打滑 / 卡死 / 辊子损坏）。
 *   想区分就量电压：万用表直流档，红笔点该脚排针、黑笔点 GND，
 *   在"闪 1"的那 3 秒窗口内 —— 3.3V ⇒ 脚是好的，问题在电机/减速箱；0V ⇒ 脚真没输出。
 *   或者把该片 DRV8833 的 OUT1/OUT2（到电机的两根线）对调再跑一遍：
 *     不动的那一段从闪1 变成闪2 ⇒ 轮子的方向性机械故障；还是闪1 不动 ⇒ PA11/IN2 侧。
 *
 * 怎么读结果：
 *   1 和 2 都转   → 两个半桥都通、PA8/PA11 两个脚都好 → 不是电气问题，是机械阻力
 *   1 不转、2 转  → PA11 → DRV8833 IN2 这条路断了（MCU 脚 / PCB 走线 / 焊点 / IN2 脚）
 *   1 转、2 不转  → PA8 → DRV8833 IN1 那条路断了
 *   1 转、3 不转  → GPIO 能推动，但定时器 CH4 出不来波形（软件问题，把结果告诉我）
 *   1 不转、3 转  → 反过来，GPIO 那条路的问题
 *   全都不转      → 那片 DRV8833 坏了，或者 IN1/IN2 接反/接到别处了
 *
 * 这一段刻意绕开 motor_set()，所以不受 invert 影响，"MCU 脚 → 电机方向"是直接对应关系。
 * 排查完把 app.h 里的 APP_RR_TEST 改回 0。
 * ==================================================================== */
extern TIM_HandleTypeDef htim1;

#define RR_SEG_MS     3000      /* 每段驱动多久 */
#define RR_GAP_MS     1500      /* 段与段之间停多久 */

static void rr_cue(uint8_t n)
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

/* in1 非 0 → PA8 拉高；in2 非 0 → PA11 拉高 */
static void rr_gpio(uint8_t in1, uint8_t in2)
{
    rr_pins_gpio();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* use_in2 非 0 → PA11 走 PWM（前进）；否则 PA8 走 PWM（后退）。CCR > ARR = 恒高 */
static void rr_pwm(uint8_t use_in2)
{
    rr_pins_af();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, use_in2 ? 0U : (MOTOR_PWM_ARR + 1U));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, use_in2 ? (MOTOR_PWM_ARR + 1U) : 0U);
}

static void rr_run(void)
{
    for (;;)
    {
        rr_cue(1);  rr_gpio(0, 1);  HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
        rr_cue(2);  rr_gpio(1, 0);  HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
        rr_cue(3);  rr_pwm(1);      HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
        rr_cue(4);  rr_pwm(0);      HAL_Delay(RR_SEG_MS);  rr_pins_af();  HAL_Delay(RR_GAP_MS);
    }
}
#endif /* APP_RR_TEST */

void app_init(void)
{
    dwt_init();         /* delay_us 要用 DWT，PS2 时序靠它 */
    ps2_init();
    teach_only_init();  /* 机械臂：舵机 + 从 Flash 读回录制（自带 TIM4 初始化）*/
    chassis_init();     /* 顺带把 8 路 PWM 起起来 */

    ps2_ok = 0;
    HAL_Delay(300);     /* 等电源稳一稳，再让电机上电 */

#if APP_RR_TEST
    rr_run();           /* 右后轮专项排查：只动右后轮，永不返回 */
#endif

#if APP_DIAG_MODE
    diag_run();         /* 排查模式：永不返回，循环跑 闪1/闪2/闪3 三段对比测试 */
#endif

#if APP_SELFTEST_ENABLE
    self_test();
#endif

    app_last_tick = HAL_GetTick();
}

void app_task(void)
{
    if ((HAL_GetTick() - app_last_tick) < APP_LOOP_MS) return;
    app_last_tick = HAL_GetTick();

    ps2_poll();


    /* ★ 示教模块必须在下面那句 if (!ps2_ok) return; 之前 ——
       否则手柄一掉线，机械臂的录制/回放也跟着停摆。 */
    teach_only_poll();

    if (!ps2_ok)
    {
        LED_OFF();      /* 灭灯 = 手柄掉线 */
        ramp_vx = ramp_vy = ramp_w = 0;
        chassis_stop(); /* 掉线/跑远 → 四轮滑行停车 */
        return;
    }

    LED_ON();

    /* START 当成急停按钮：按住就停，方便慌张的时候一把按住 */
    if (ps2_btn1 & PS2_BTN_START)
    {
        ramp_vx = ramp_vy = ramp_w = 0;
        chassis_stop();
        return;
    }

    /* 速度档：默认快速档，按住 L1 切慢速档（对位、贴近目标的时候用） */
    int16_t speed_max = (ps2_btn2 & PS2_BTN_L1) ? APP_SPEED_SLOW : APP_SPEED_MAX;
    if (speed_max < APP_SPEED_MIN) speed_max = APP_SPEED_MIN;

    /* 左摇杆 = 平移：推哪个方向车就往哪个方向走（含斜向和横移） */
    int16_t vx, vy;
    stick_to_vector(ps2_lx, ps2_ly, speed_max, &vx, &vy);

    /* 右摇杆左右 = 原地自转（左推 = 左转 = 从上往下看逆时针） */
    int16_t w = (int16_t)(-stick_to_speed(ps2_rx, speed_max));

    /* 三个自由度各自过一遍斜坡，再交给底盘层做麦轮逆解 */
    ramp_vx = ramp_to(ramp_vx, vx);
    ramp_vy = ramp_to(ramp_vy, vy);
    ramp_w  = ramp_to(ramp_w,  w);

    chassis_drive(ramp_vx, ramp_vy, ramp_w);
}
