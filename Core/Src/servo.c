/*
servo.c —— 4 路舵机 / 夹持臂控制

硬件：PB6~PB9 = TIM4_CH1~CH4
      TIM4 PSC=71, ARR=19999 → 72MHz/72/20000 = 50Hz，每 tick = 1µs
      所以 CCR 直接写脉宽微秒数即可。

本模块提供两种用法：
  1. servo_set(通道, 脉宽us)   —— 原始接口，直接写 CCR，不缓动
  2. arm_* 系列               —— 角度 + 非阻塞缓动，给夹持臂用

缓动为什么必须做：MG995 在 7.4V 下堵转电流可达 2A 以上，
而官方 Mini 5V 降压模块余量有限。4 路同时瞬时跳变会把 5V 拉垮，
表现就是 STM32 复位、手柄失联。所以所有角度变化都按"度/秒"限速。
*/
#include "main.h"
#include "servo.h"

/* ===========================================================================
 * ★ TIM4 由本模块【自己初始化】，不依赖工程里有没有 htim4
 * ---------------------------------------------------------------------------
 * 为什么这么做：这个模块要能直接丢进别人的工程用。
 * 如果写 `extern TIM_HandleTypeDef htim4;`，那对方必须先在 CubeMX 里
 * 把 TIM4 配成 50Hz、把 PB6~PB8 配成复用推挽 —— 少一样就编译不过或不动。
 *
 * 现在改成本模块自带：
 *   · 自己开 TIM4 时钟
 *   · 自己把 PB6/PB7/PB8 配成复用推挽
 *   · 自己按 72MHz → PSC=71 / ARR=19999 配成 50Hz（1 计数 = 1µs）
 *   · 自己启动 3 路 PWM
 *
 * 好处：拷进任何 STM32F103 工程都能直接用，
 *       也不会和对方 CubeMX 生成的 htim4 抢名字。
 * =========================================================================== */

#define SERVO_TIM          TIM4
#define SERVO_TIM_CLK_EN() __HAL_RCC_TIM4_CLK_ENABLE()
#define SERVO_GPIO_CLK_EN() __HAL_RCC_GPIOB_CLK_ENABLE()

static TIM_HandleTypeDef htim_servo;      /* 本模块私有的句柄，对外不可见 */

/* 一次性把 TIM4 配好并启动（幂等，重复调用无副作用） */
static void servo_hw_init(void)
{
    static uint8_t inited = 0;

    GPIO_InitTypeDef        gpio = {0};
    TIM_OC_InitTypeDef      oc   = {0};

    if (inited) return;
    inited = 1;

    SERVO_TIM_CLK_EN();
    SERVO_GPIO_CLK_EN();

    /* ---- PB6 / PB7 / PB8 → 复用推挽（TIM4_CH1~CH3）----
       ★ 这里不配 PB9：CH4 是留给双夹爪备用的，没用到就不要占引脚。 */
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ---- 50Hz：72MHz ÷ 72 = 1MHz → 1 个计数 = 1µs；20000 个计数 = 20ms ---- */
    htim_servo.Instance               = SERVO_TIM;
    htim_servo.Init.Prescaler         = 71;          /* 72MHz/(71+1) = 1MHz */
    htim_servo.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_servo.Init.Period            = 19999;       /* 20000µs = 20ms = 50Hz */
    htim_servo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim_servo);

    /* ---- 4 路 PWM 通道（初始脉宽 1500µs = 中位，免得一上电就是 0）---- */
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse      = 1500;

    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_4);

    /* ---- 启动 3 路（CH4 空出，不启动，免得占着 PB9）---- */
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_3);
}

/* 兼容写法：下面统一用这个别名访问寄存器 */
#define htim4  htim_servo

/* ==========================================================================
 * 标定姿态 —— ⚠️ 全部是【占位值，必须重新标定】★ 见 servo.h 那段说明
 * ==========================================================================
 * 机构改成"3 关节两级（大臂 / 肘 / 夹爪）"之后，旧尺寸总表的角度全部失效，
 * 直接用会**把臂顶到机械限位上**。所以这里统一取 90°（中位）：
 * 中位离两端最远，就算误触发也不会撞。
 *
 * 夹爪那一列是"舵机角度"，由 GRIP_OPEN / GRIP_CLOSE 派生
 * （= grip_set() 用的同一套标定值，见 servo.h 那段说明），所以这两个不用手填。
 *
 * 【填真实值的方法】用 teach.c 的示教录制（推荐）或 build/ServoCalib 标定固件。
 * ========================================================================== */
const Pose_t POSE_STOW = { 90, 90, GRIP_CLOSE };   /* 收纳/运输 ← 自动收回的目标 */
const Pose_t POSE_GRAB = { 90, 90, GRIP_OPEN  };   /* 抓取（占位）*/
const Pose_t POSE_LIFT = { 90, 90, GRIP_CLOSE };   /* 抬起过渡（占位）*/
const Pose_t POSE_FEED = { 90, 90, GRIP_CLOSE };   /* 投料位置（占位）*/

/* 上电位置：中位。三个关节都在行程中间，离两端最远，最安全。
   （想改成"上电去收纳姿态"，把 servo.h 的 ARM_POWERON_TO_STOW 设成 1，
     但**必须先标定 POSE_STOW**。）*/
static const Pose_t POSE_SAFE = { 90, 90,
    (int16_t)((SERVO_GRIP_ANGLE_OPEN + SERVO_GRIP_ANGLE_CLOSE) / 2) };

/* 每通道的脉宽与角度量程。索引 = 通道号 - 1。
 * 四个舵机都是 180° 型号，所以现在一样；
 * 保留这个结构是为了以后单独改某个通道（比如换 270° 舵机）。
 * 数组按 SERVO_CH_MAX(4) 开，但只用前 SERVO_CH_COUNT(3) 个。 */
static const uint16_t pulse_min[SERVO_CH_MAX] = {
    SERVO_PULSE_MIN_180, SERVO_PULSE_MIN_180,
    SERVO_PULSE_MIN_180, SERVO_PULSE_MIN_180
};
static const uint16_t pulse_max[SERVO_CH_MAX] = {
    SERVO_PULSE_MAX_180, SERVO_PULSE_MAX_180,
    SERVO_PULSE_MAX_180, SERVO_PULSE_MAX_180
};
static const int16_t  angle_max[SERVO_CH_MAX] = {
    SERVO_ANGLE_MAX_180, SERVO_ANGLE_MAX_180,
    SERVO_ANGLE_MAX_180, SERVO_ANGLE_MAX_180
};

/* 关节软限位表，索引 = 通道号-1。
   第 4 项（CH4/PB9）空出来，给个全量程占位，反正不用。 */
static const int16_t limit_min[SERVO_CH_MAX] = {
    ARM_MIN, ELBOW_MIN, 0, 0
};
static const int16_t limit_max[SERVO_CH_MAX] = {
    ARM_MAX, ELBOW_MAX, SERVO_ANGLE_MAX_180, SERVO_ANGLE_MAX_180
};

typedef struct {
    int16_t cur;        /* 当前角（缓动中的实时值）*/
    int16_t target;     /* 目标角 */
    int16_t step;       /* 每 20ms 走多少度 */
    uint8_t hard;       /* 1 = 跳过软限位（标定/调试用）*/
} Joint_t;

static Joint_t joint[SERVO_CH_MAX];

/* 运行时可改的"收纳姿态"（示教用）。默认 = POSE_STOW 的占位值。
   放在这里是为了让 arm_retract() 能拿到它，而 servo.c 不用反过来依赖 teach.c。 */
static Pose_t stow_pose = { 90, 90, GRIP_CLOSE };

/* ---------------------------------------------------------------------------
 * 基础：脉宽 <-> 角度换算（按通道各自的量程）
 *
 * ★ 这里为什么要"+半个除数"（四舍五入）
 * ----------------------------------------------------------------
 * 原来两个函数都用整数除法直接截断，结果**不能往返**：
 *     deg2us(5)  = 1000 + 5*1000/180 = 1027   （实际 1027.8，截断成 1027）
 *     us2deg(1027) = (27*180)/1000   = 4      ← 变成 4° 了，少了 1°
 * 全量程每 5° 检查一遍，有 32 个点对不上，最大误差 1°。
 *
 * 影响：servo_set() 会用 us2deg() 回写内部当前角度，
 *       来回调用会让目标角度每次滑掉约 1°，摇杆微调时手感发飘。
 *
 * 修法：两边都四舍五入，往返就一致了。
 *      （这是"定点运算要小心截断方向"的经典例子）
 * ---------------------------------------------------------------- */
static uint16_t deg2us(uint8_t ch, int16_t deg)
{
    uint8_t  i = (uint8_t)(ch - 1);
    int32_t  span;
    int32_t  us;

    if (i >= SERVO_CH_MAX) i = 0;

    if (deg < 0)             deg = 0;
    if (deg > angle_max[i])  deg = angle_max[i];

    span = (int32_t)pulse_max[i] - (int32_t)pulse_min[i];

    /* 加半个除数 = 四舍五入，避免截断导致往返不一致 */
    us = (int32_t)pulse_min[i]
       + ((int32_t)deg * span + angle_max[i] / 2) / angle_max[i];

    if (us < (int32_t)pulse_min[i]) us = (int32_t)pulse_min[i];
    if (us > (int32_t)pulse_max[i]) us = (int32_t)pulse_max[i];

    return (uint16_t)us;
}

static int16_t us2deg(uint8_t ch, uint16_t us)
{
    uint8_t  i = (uint8_t)(ch - 1);
    int32_t  span;
    int32_t  d;

    if (i >= SERVO_CH_MAX) i = 0;

    if (us < pulse_min[i]) us = pulse_min[i];
    if (us > pulse_max[i]) us = pulse_max[i];

    span = (int32_t)pulse_max[i] - (int32_t)pulse_min[i];
    if (span == 0) return 0;               /* 防除零 */

    /* 同样四舍五入 */
    d = (((int32_t)us - (int32_t)pulse_min[i]) * angle_max[i] + span / 2) / span;

    if (d < 0)              d = 0;
    if (d > angle_max[i])   d = angle_max[i];

    return (int16_t)d;
}

static void write_ccr(uint8_t ch, uint16_t us)
{
    switch (ch)
    {
        case CH_ARM:   htim4.Instance->CCR1 = us; break;   /* PB6 */
        case CH_ELBOW: htim4.Instance->CCR2 = us; break;   /* PB7 */
        case CH_GRIP:  htim4.Instance->CCR3 = us; break;   /* PB8 */
        case CH_SPARE: htim4.Instance->CCR4 = us; break;   /* PB9 空出，留着 */
        default: break;
    }
}

static int16_t clamp_angle(uint8_t ch, int16_t deg, uint8_t hard)
{
    uint8_t i = (uint8_t)(ch - 1);

    if (i >= SERVO_CH_MAX) return deg;
    if (hard)  return deg;

    if (deg < limit_min[i]) deg = limit_min[i];
    if (deg > limit_max[i]) deg = limit_max[i];
    return deg;
}

/* ---------------------------------------------------------------------------
 * 原始接口：直接写脉宽，不缓动（保留给测试/兼容）
 * ------------------------------------------------------------------------- */
void servo_set(uint8_t no, uint16_t us)
{
    if (no < 1 || no > SERVO_CH_COUNT) return;

    /* 夹到该通道自己的脉宽范围 */
    if (us < pulse_min[no - 1]) us = pulse_min[no - 1];
    if (us > pulse_max[no - 1]) us = pulse_max[no - 1];

    write_ccr(no, us);

    /* 同步内部状态，免得后续 arm_* 又把它拽回去 */
    joint[no - 1].cur    = us2deg(no, us);
    joint[no - 1].target = joint[no - 1].cur;
}

/* ---------------------------------------------------------------------------
 * 标定专用接口 —— 越过软限位和脉宽夹紧，用来"探边界"
 * ⚠️ 只在 CAR_SERVO_ONLY 测试模式里用
 * ------------------------------------------------------------------------- */
void servo_cal_pulse(uint8_t ch, uint16_t us)
{
    if (ch < 1 || ch > SERVO_CH_COUNT) return;

    /* 这里故意不夹 pulse_min/max：标定就是要试出安全范围到底在哪。
       但也不能太离谱，做个 300~2700us 的兜底保护。 */
    if (us < 300)  us = 300;
    if (us > 2700) us = 2700;

    write_ccr(ch, us);

    joint[ch - 1].cur    = us2deg(ch, us);
    joint[ch - 1].target = joint[ch - 1].cur;
}

void servo_cal_angle(uint8_t ch, int16_t deg)
{
    uint8_t prev_hard;

    if (ch < 1 || ch > SERVO_CH_COUNT) return;

    /* 按该通道自己的角度上限夹（目前四个通道都是 180）*/
    if (deg < 0)                deg = 0;
    if (deg > angle_max[ch - 1]) deg = angle_max[ch - 1];

    /* -------------------------------------------------------------------
     * ★ 只【本次】跳过软限位，写完立刻恢复原来的模式。
     *
     * 原来是 `hard = 1;` 置上去就不管了 —— 后果是：
     *   只要调用方调过一次本函数（示教固件**每帧都在调**），
     *   该通道的软限位就【永久失效】，
     *   之后 arm_set_target() 经 clamp_angle() 直接放行，
     *   一路顶到机械限位（大舵机堵转发热 / 小舵机崩塑料齿）。
     *
     * 现在做成"临时旁路"：本来就处于标定模式（hard 已为 1）的调用方，
     * 调用后仍然是 1，行为不变；正常调用方则自动恢复保护。
     * ------------------------------------------------------------------- */
    prev_hard = joint[ch - 1].hard;

    joint[ch - 1].hard   = 1;            /* 本次跳过软限位 */
    joint[ch - 1].cur    = deg;
    joint[ch - 1].target = deg;

    write_ccr(ch, deg2us(ch, deg));

    joint[ch - 1].hard   = prev_hard;    /* ★ 恢复 */
}

int16_t servo_cal_get(uint8_t ch)
{
    if (ch < 1 || ch > SERVO_CH_COUNT) return 0;
    return joint[ch - 1].target;
}

/* ---------------------------------------------------------------------------
 * 夹持臂接口
 * ------------------------------------------------------------------------- */
void arm_init(void)
{
    /* -------------------------------------------------------------------
     * 上电先摆到"中位"，并且**把内部状态也一起设成中位**。
     *
     * ★ 这里有个很容易踩的坑：joint[] 是静态变量，上电全是 0。
     *   如果只写 CCR 不同步 joint[].cur/target，那么主循环第一次 servo_poll()
     *   会认为"当前在 0°、目标在 90°"，于是**先把舵机一把拽到 0°**，
     *   再从 0° 慢慢缓动上来 —— 表现就是**上电瞬间猛甩一下**。
     *   所以 cur 和 target 必须在这里一起设成中位。
     * ------------------------------------------------------------------- */
    for (uint8_t ch = 1; ch <= SERVO_CH_COUNT; ch++)
    {
        int16_t mid = (int16_t)(angle_max[ch - 1] / 2);

        /* ★ 把上电位置【夹进软限位区间】。
           只用 angle_max/2（恒为 90）是不够的：如果标定后把某个关节的
           限位收紧到不含 90（例如大臂改成 [95,175]），上电瞬间就会把
           关节驱到一个【被禁止】的角度 —— 正是本项目"上电即顶限位"的老坑。
           夹一下之后，不管限位怎么改，上电位置永远是合法的。 */
        if (mid < limit_min[ch - 1]) mid = limit_min[ch - 1];
        if (mid > limit_max[ch - 1]) mid = limit_max[ch - 1];

        write_ccr(ch, deg2us(ch, mid));

        joint[ch - 1].cur    = mid;
        joint[ch - 1].target = mid;
    }

    /* ★ 硬件初始化（时钟 + PB6~PB8 复用 + 50Hz + 启动 PWM）都由本模块自己做，
       不依赖工程里的 htim4 或 CubeMX 配置 */
    servo_hw_init();

    /* 每 20ms 的角度步进 = 度/秒 ÷ 50（主循环 50Hz）*/
    joint[CH_ARM   - 1].step = ARM_SPEED   / 50;
    joint[CH_ELBOW - 1].step = ELBOW_SPEED / 50;
    joint[CH_GRIP  - 1].step = GRIP_SPEED  / 50;

    for (uint8_t i = 0; i < SERVO_CH_COUNT; i++)
    {
        if (joint[i].step < 1) joint[i].step = 1;   /* 至少每 20ms 动 1° */
        joint[i].hard = 0;
    }

    /* -------------------------------------------------------------------
     * 上电去哪？
     *
     * ★ 默认是"保持中位"（POSE_SAFE = 90/90/90），**不是**回收纳姿态。
     *   原因：新机构的 POSE_STOW 还没标定，占位值和真实机械位置没关系，
     *   上电就冲过去可能直接把臂顶到限位上。
     *
     *   等 POSE_STOW 标定好之后，把 servo.h 的 ARM_POWERON_TO_STOW 改成 1，
     *   就会恢复"上电缓动到收纳姿态"（检录定妆 / 每局开局的姿态）。
     * ------------------------------------------------------------------- */
#if ARM_POWERON_TO_STOW
    arm_goto_pose(&POSE_STOW);
#else
    arm_goto_pose(&POSE_SAFE);
#endif
}

void arm_set_target(uint8_t ch, int16_t deg)
{
    uint8_t i;

    if (ch < 1 || ch > SERVO_CH_COUNT) return;
    i = (uint8_t)(ch - 1);

    joint[i].target = clamp_angle(ch, deg, joint[i].hard);
}

void arm_set_target_raw(uint8_t ch, int16_t deg)
{
    uint8_t i;

    if (ch < 1 || ch > SERVO_CH_COUNT) return;
    i = (uint8_t)(ch - 1);

    joint[i].target = clamp_angle(ch, deg, 1);   /* hard = 1，跳过软限位 */
}

void arm_jog(uint8_t ch, int16_t delta)
{
    uint8_t i;

    if (ch < 1 || ch > SERVO_CH_COUNT) return;
    i = (uint8_t)(ch - 1);

    /* 基于"目标"而不是"当前"，连续按键才能累积，不会自己追自己 */
    arm_set_target(ch, (int16_t)(joint[i].target + delta));
}

int16_t arm_get_angle(uint8_t ch)
{
    if (ch < 1 || ch > SERVO_CH_COUNT) return 0;
    return joint[ch - 1].cur;
}

int16_t arm_get_target(uint8_t ch)
{
    if (ch < 1 || ch > SERVO_CH_COUNT) return 0;
    return joint[ch - 1].target;
}

uint8_t arm_is_moving(void)
{
    for (uint8_t i = 0; i < SERVO_CH_COUNT; i++)
    {
        if (joint[i].cur != joint[i].target) return 1;
    }
    return 0;
}

void arm_goto_pose(const Pose_t *p)
{
    if (p == 0) return;

    arm_set_target(CH_ARM,   p->arm);
    arm_set_target(CH_ELBOW, p->elbow);
    arm_set_target(CH_GRIP,  p->grip);
}

/* ★ 自动收回：缓动回收纳姿态（stow_pose）。
 *   回放结束后自动调它（teach.c 的 TEACH_AUTO_STOW），手柄 △ 也绑它。
 *
 *   ⚠️ 三个关节是**同时**开始缓动的，各自按自己的速度走
 *      （大臂慢、肘快），所以实际会形成"肘先到、大臂后到"的自然先后。
 *      如果你的机构在收回时肘会撞到大臂，别用这个"一键收回"，
 *      改用示教录制把收回动作也录进去 —— 那是精确的。 */
void arm_retract(void)
{
    arm_goto_pose(&stow_pose);
}

void arm_open_grip(void)  { arm_set_target(CH_GRIP, GRIP_OPEN);  }
void arm_close_grip(void) { arm_set_target(CH_GRIP, GRIP_CLOSE); }

/* ---- 运行时可改的"收纳姿态" ------------------------------------------- */
void arm_set_stow(const Pose_t *p)
{
    if (p == 0) return;
    stow_pose = *p;
}

Pose_t arm_get_stow(void)
{
    return stow_pose;
}

/* ===========================================================================
 * 夹爪：单舵机齿轮齿条（默认） / 双舵机对开（可选）
 * ===========================================================================
 * 两种模式都用同一个 0~100 的开合量接口：
 *     grip_set(0)   = 全开
 *     grip_set(100) = 全合（夹紧）
 * 上层代码（chapter.c、手柄按键）不需要区分模式。
 *
 * 单舵机模式（SERVO_GRIP_DUAL=0）：一个通道，角度线性映射到开合量。
 * 双舵机模式（SERVO_GRIP_DUAL=1）：两个通道，镜像方向，一起缓动。
 * =========================================================================== */

/* 把开合量 0~100 线性映射成某个通道的角度 */
#if SERVO_GRIP_DUAL
static int16_t grip_pct_to_deg(int16_t pct, int16_t a_open, int16_t a_close, uint8_t invert)
#else
static int16_t grip_pct_to_deg(int16_t pct)
#endif
{
#if SERVO_GRIP_DUAL
    int32_t deg;
    int16_t a_open_l  = a_open;
    int16_t a_close_l = a_close;

    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    deg = (int32_t)a_open_l + ((int32_t)a_close_l - (int32_t)a_open_l) * pct / 100;

    if (invert) deg = SERVO_ANGLE_MAX_180 - deg;
#else
    int32_t deg;

    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    deg = (int32_t)SERVO_GRIP_ANGLE_OPEN
        + ((int32_t)SERVO_GRIP_ANGLE_CLOSE - (int32_t)SERVO_GRIP_ANGLE_OPEN) * pct / 100;

    if (SERVO_GRIP_INVERT) deg = SERVO_ANGLE_MAX_180 - deg;
#endif

    if (deg < 0)   deg = 0;
    if (deg > SERVO_ANGLE_MAX_180) deg = SERVO_ANGLE_MAX_180;

    return (int16_t)deg;
}

void grip_set(int16_t pct)
{
    /* -------------------------------------------------------------------
     * ★ GRIP_PCT_HOLD(-1) 的语义是"这一步不动夹爪"（动作序列里用），
     *   必须【原样忽略】。原来下面那句 `if (pct < 0) pct = 0;` 会把它
     *   夹成 0，而 0 = 全开 —— 语义正好相反（本该不动，结果把夹爪张开了）。
     * ------------------------------------------------------------------- */
    if (pct == GRIP_PCT_HOLD) return;

    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

#if SERVO_GRIP_DUAL
    /* ★ 不需要动 joint[].hard —— arm_set_target_raw() 本身就跳过软限位。
       原来这里把 hard 置 1 就不管了，会让两个夹爪舵机的软限位【永久失效】
       （和 servo_cal_angle 是同一个坑，见那边的注释）。 */
    arm_set_target_raw(SERVO_GRIP_A_CH,
        grip_pct_to_deg(pct, SERVO_GRIP_A_ANGLE_OPEN, SERVO_GRIP_A_ANGLE_CLOSE,
                        SERVO_GRIP_A_INVERT));

    arm_set_target_raw(SERVO_GRIP_B_CH,
        grip_pct_to_deg(pct, SERVO_GRIP_B_ANGLE_OPEN, SERVO_GRIP_B_ANGLE_CLOSE,
                        SERVO_GRIP_B_INVERT));
#else
    arm_set_target_raw(SERVO_GRIP_CH, grip_pct_to_deg(pct));
#endif
}

void grip_open(void)  { grip_set(0);   }
void grip_close(void) { grip_set(100); }

void grip_toggle(void)
{
    /* 开合量过半就当它是"合"，再去切换 */
    if (grip_get() >= 50) grip_open();
    else                  grip_close();
}

int16_t grip_get(void)
{
    int16_t deg;
    int32_t pct;

#if SERVO_GRIP_DUAL
    int32_t span;
    deg = joint[SERVO_GRIP_A_CH - 1].cur;
    if (SERVO_GRIP_A_INVERT) deg = (int16_t)(SERVO_ANGLE_MAX_180 - deg);
    span = (int32_t)SERVO_GRIP_A_ANGLE_CLOSE - (int32_t)SERVO_GRIP_A_ANGLE_OPEN;
    if (span == 0) return 0;
    pct = ((int32_t)deg - (int32_t)SERVO_GRIP_A_ANGLE_OPEN) * 100 / span;
#else
    int32_t span;
    deg = joint[SERVO_GRIP_CH - 1].cur;
    if (SERVO_GRIP_INVERT) deg = (int16_t)(SERVO_ANGLE_MAX_180 - deg);
    span = (int32_t)SERVO_GRIP_ANGLE_CLOSE - (int32_t)SERVO_GRIP_ANGLE_OPEN;
    if (span == 0) return 0;
    pct = ((int32_t)deg - (int32_t)SERVO_GRIP_ANGLE_OPEN) * 100 / span;
#endif

    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    return (int16_t)pct;
}

uint8_t grip_is_moving(void)
{
#if SERVO_GRIP_DUAL
    if (joint[SERVO_GRIP_A_CH - 1].cur != joint[SERVO_GRIP_A_CH - 1].target) return 1;
    if (joint[SERVO_GRIP_B_CH - 1].cur != joint[SERVO_GRIP_B_CH - 1].target) return 1;
#else
    if (joint[SERVO_GRIP_CH - 1].cur != joint[SERVO_GRIP_CH - 1].target) return 1;
#endif
    return 0;
}

/* ---------------------------------------------------------------------------
 * 每轮主循环调用（50Hz）——非阻塞缓动
 * ------------------------------------------------------------------------- */
void servo_poll(void)
{
    for (uint8_t i = 0; i < SERVO_CH_COUNT; i++)
    {
        /* ★ 用 int32_t 算差值：target/cur 都被夹在 [0,180]，
           所以 int16_t 其实不会溢出；但写成 int32 后
           `-d` 也永远安全（避免"万一有人把夹紧去掉"就踩到 INT16_MIN 的 UB）。 */
        int32_t d = (int32_t)joint[i].target - (int32_t)joint[i].cur;

        if (d == 0) continue;

        if (d > 0)
        {
            joint[i].cur = (int16_t)((d > joint[i].step) ? joint[i].cur + joint[i].step
                                                         : joint[i].target);
        }
        else
        {
            joint[i].cur = (int16_t)((-d > joint[i].step) ? joint[i].cur - joint[i].step
                                                          : joint[i].target);
        }

        write_ccr((uint8_t)(i + 1), deg2us((uint8_t)(i + 1), joint[i].cur));
    }
}

/* ---------------------------------------------------------------------------
 * 兼容旧接口：启动 PWM 并全部摆到安全位置
 * ------------------------------------------------------------------------- */
void servo_start(void)
{
    arm_init();
}
