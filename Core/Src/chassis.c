/*
chassis.c —— 把 vx/vy/omega 三个自由度的速度 → 四个轮子的速度 → 写入 motor.c
    chassis_drive(): vx/vy/omega → 四轮速度 → 写入 motor.c
    chassis_stop(): 让四个轮子都滑行停车
    chassis_forward/backward/turn_left/turn_right/strafe_left/strafe_right(): 方便的单自由度接口
    chassis_wheel(): 单轮测试用（上电自检）：只动指定的那个轮子
    chassis_init(): 初始化，顺便把 motor.c 的定时器 PWM 开起来
*/
#include "main.h"
#include "chassis.h"
#include "motor.h"

/*
从"停"变成"要动"的那一下，先给满占空比踹 CHASSIS_KICK_MS 毫秒，再落回目标速度。
为什么需要：实测这颗电机静摩擦偏大（20kHz 下 40% 占空比根本带不动），
尤其右后轮有一个方向直接给 60% 会先"愣"一下才转、甚至愣住不动。
满占空比踹一脚能可靠冲破静摩擦，之后维持所需的占空比就低得多。
设成 0 就是关闭这个功能。
*/
#define CHASSIS_KICK_MS    80

/* 上一拍发给四个轮子的指令，用来判断"这一拍是不是从静止起步"。
 * 下标 0 占位，让轮号 = 下标；只在 chassis.c 内部用。 */
static int16_t last_cmd[5];

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

void chassis_init(void)
{
    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++) last_cmd[w] = 0;
    motor_init();
}

/* 把四个轮子的目标速度真正写下去，起步时顺带踹一脚 */
static void chassis_apply(const int16_t cmd[5])
{
    uint8_t need_kick = 0;

    if(CHASSIS_KICK_MS > 0) 
        for(uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++) 
            if(cmd[w] != 0 && last_cmd[w] == 0) { need_kick = 1; break; }
    // 如果需要给个突然高电平，并且上次这个轮子是静止的，就给它一顺便的高电压作为启动

    if (need_kick)
    {
        for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++)
            if (cmd[w] != 0)
                motor_set(w, (cmd[w] > 0) ? MOTOR_SPEED_MAX : -MOTOR_SPEED_MAX);
                /* 方向和目标一致，占空比直接拉满 */
        HAL_Delay(CHASSIS_KICK_MS);
    }

    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++)
    {
        motor_set(w, cmd[w]);
        last_cmd[w] = cmd[w];
    }
}

void chassis_drive(int16_t vx, int16_t vy, int16_t omega)
{
    int16_t cmd[5];
    int32_t fl, fr, rl, rr, maxabs;
    int32_t v = (int32_t)vy * CHASSIS_STRAFE_SIGN;

    fl = (int32_t)vx - v - (int32_t)omega;
    fr = (int32_t)vx + v + (int32_t)omega;
    rl = (int32_t)vx + v - (int32_t)omega;
    rr = (int32_t)vx - v + (int32_t)omega;

    /* 三个自由度一起给满时，四轮里最大的那个会超量程。
       这时候按同一个比例把四个轮子一起缩小 —— 不能单独截断，
       否则前进 / 横移 / 自转的方向比例就被改了，车会走歪。 */
    maxabs = iabs32(fl);
    if (iabs32(fr) > maxabs) maxabs = iabs32(fr);
    if (iabs32(rl) > maxabs) maxabs = iabs32(rl);
    if (iabs32(rr) > maxabs) maxabs = iabs32(rr);

    if (maxabs > CHASSIS_SPEED_MAX)
    {
        fl = fl * CHASSIS_SPEED_MAX / maxabs;
        fr = fr * CHASSIS_SPEED_MAX / maxabs;
        rl = rl * CHASSIS_SPEED_MAX / maxabs;
        rr = rr * CHASSIS_SPEED_MAX / maxabs;
    }

    cmd[MOTOR_FL] = (int16_t)fl;
    cmd[MOTOR_FR] = (int16_t)fr;
    cmd[MOTOR_RL] = (int16_t)rl;
    cmd[MOTOR_RR] = (int16_t)rr;

    chassis_apply(cmd);
}

void chassis_stop(void)
{
    /* 清零 last_cmd，下次再动的时候才会重新踹一脚 */
    for (uint8_t w = MOTOR_FL; w <= MOTOR_RR; w++) last_cmd[w] = 0;
    motor_stop_all();
}

void chassis_forward(int16_t speed)        { chassis_drive(speed, 0, 0); }
void chassis_backward(int16_t speed)       { chassis_drive((int16_t)(-speed), 0, 0); }
void chassis_turn_left(int16_t speed)      { chassis_drive(0, 0, speed); }
void chassis_turn_right(int16_t speed)     { chassis_drive(0, 0, (int16_t)(-speed)); }
void chassis_strafe_left(int16_t speed)    { chassis_drive(0, (int16_t)(-speed), 0); }
void chassis_strafe_right(int16_t speed)   { chassis_drive(0, speed, 0); }

/* 单轮测试用（上电自检）：只动指定的那个轮子 */
void chassis_wheel(uint8_t wheel, int16_t speed)
{
    int16_t cmd[5] = { 0, 0, 0, 0, 0 };

    if (wheel < MOTOR_FL || wheel > MOTOR_RR) return;
    if (speed >  CHASSIS_SPEED_MAX) speed =  CHASSIS_SPEED_MAX;
    if (speed < -CHASSIS_SPEED_MAX) speed = -CHASSIS_SPEED_MAX;

    cmd[wheel] = speed;
    chassis_apply(cmd);
}
