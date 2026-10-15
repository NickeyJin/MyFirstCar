/*
 * app.c —— 应用层（上电自检 + PS2 手柄全向控制）
 *
 * 职责：
 *   1. 上电自检（APP_SELFTEST_ENABLE）：四个轮子挨个正转一下、反转一下，
 *      再四轮一起前后走、横移一步 —— 不用手柄也能一眼看出"哪个轮子没接 /
 *      哪个接反了 / 横移方向对不对"。
 *   2. 主循环：读 PS2 → 左摇杆平移、右摇杆自转 → 过一遍加减速斜坡 →
 *      交给 chassis 层做麦轮逆运动学。
 *
 * 安全策略：只要手柄没数据（ps2_ok == 0），立刻四轮滑行停车；
 *           按住 START 当作急停按钮。
 *
 * 开发期的排查代码（PWM 占空比对比、右后轮专项）在 diag.c，开关在 diag.h，
 * 正常跑车时两个开关都是 0，那些代码根本不参与编译。
 */
#include "main.h"
#include "app.h"
#include "board.h"
#include "chassis.h"
#include "diag.h"
#include "dwt_delay.h"
#include "motor.h"
#include "ps2.h"
#include "teach_only.h"   /* 机械臂示教录制/回放 */

/* 自检参数 */
#define TEST_SPEED       600    /* 自检转速：实测这颗电机 40% 才起步，自检用 60% 才看得清 */
#define TEST_RUN_MS      300    /* 每个动作持续多久 */
#define TEST_GAP_MS      150    /* 动作之间的停顿 */
#define TEST_WHEEL_GAP   250    /* 两个轮子之间的停顿 */

static uint32_t app_last_tick;

/* ==================== 摇杆 → 速度 ====================
 * PS2 的轴值 0~255，中点是 0x80。分两步处理：
 *   1) stick_raw()：抠掉死区，输出 -115 ~ +115（保留符号和幅度）
 *   2) 再按 speed_max 归一化：摇杆幅值映射到 APP_SPEED_MIN ~ speed_max，
 *      而不是 0 ~ speed_max —— 实测这颗电机低占空比带不动，从 0 映射的话，
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
static void test_wheel(uint8_t wheel)
{
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

    test_wheel(MOTOR_FL);
    test_wheel(MOTOR_FR);
    test_wheel(MOTOR_RL);
    test_wheel(MOTOR_RR);

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

void app_init(void)
{
    dwt_init();         /* delay_us 要用 DWT，PS2 时序靠它 */
    ps2_init();
    teach_only_init();  /* 机械臂：舵机 + 从 Flash 读回录制（自带 TIM4 初始化）*/
    chassis_init();     /* 顺带把 8 路 PWM 起起来 */

    ps2_ok = 0;
    HAL_Delay(300);     /* 等电源稳一稳，再让电机上电 */

#if APP_RR_TEST
    diag_rr_run();      /* 右后轮专项排查：只动右后轮，永不返回 */
#endif

#if APP_DIAG_MODE
    diag_run();         /* PWM 对比测试：永不返回 */
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
