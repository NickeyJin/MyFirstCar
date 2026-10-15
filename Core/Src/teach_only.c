/*
teach_only.c —— 机械臂示教录制 / 回放（单文件，带 Flash 保存）

===============================================================================
干什么：**人手做一遍 → 按 □ 存进芯片 → 以后上电按 START 就能重放。**
===============================================================================

【边界：这个文件只管机械臂】

    ★ 麦轮底盘是队员负责的模块 —— 本文件**完全不碰**：
        · 不初始化 TIM1/TIM2/TIM3（那是电机的）
        · 一个摇杆都不读（两个摇杆归底盘）
        · 不调用任何 motor_* / mecanum_* 函数

    机械臂只用 TIM4（PB6/PB7/PB8）和一个 PC13 灯。
    两个摇杆空着不动 —— 合并到底盘代码里的时候，那边自己读。

    合并接口见 teach_only.h：调 teach_only_init() 和 teach_only_poll()。

【三个状态】
    空闲 ──SELECT──► 录制中 ──SELECT──► 空闲
    空闲 ──START───► 回放中 ──START/放完──► 空闲

【数据存哪】
    RAM ：每 40ms 一帧，每帧 3 个角度（0~180 塞进 1 字节）
          512 帧 × 3 = **1536 字节**，够 20.5 秒
    Flash：按 □ 写进芯片最后 2 页（0x0800F800），**掉电不丢**
          上电自动读回来，直接就能按 START 重放

    为什么 40ms（25Hz）？
      主循环是 50Hz（20ms 一圈），40ms **正好是 2 圈**，
      录制和回放的节奏天然一致，不会漂。

【按键表】（摇杆留给底盘，所以机械臂全在十字键和按键上）
    左摇杆 / 右摇杆 左右   ⚠️ 归底盘，本模块不读
    ─────────────────────────────────────────────
    L1 / R1            切换选中的关节（大臂 → 肘 → 夹爪 → 大臂）
    十字键 上 / 下      选中关节 ±步长（**按住连续动**，方便教动作）
    L2 / R2            步长 1° ↔ 5°
    十字键 左 / 右      夹爪 张开 / 夹紧
    ○ 圆圈             选中关节回中位（90°）
    △ 三角             三个关节全部回中位
    □ 方块             **存进 Flash**
    × 叉叉             从 Flash 读回来
    ─────────────────────────────────────────────
    SELECT             开始录制 / 结束录制
    START              回放 / 停止回放

【PC13 灯】
    常亮           待命（可以开始）
    很慢闪 500ms   正在录制
    快闪 80ms      正在回放
    慢闪 250ms     手柄掉线
    连闪 3 下      存 Flash 成功
    连闪 2 下      读到 Flash 内容 / 上电自动载入成功
    闪 1 下        存/读失败
=============================================================================== */
#include "main.h"
#include "dwt_delay.h"
#include "ps2.h"
#include "servo.h"
#include "teach_only.h"

/* ==================== 配置 ==================== */
#define JOINT_N       3      /* 录几个关节：大臂 / 肘 / 夹爪 */
#define SAMPLE_MS     40     /* 每帧间隔。40ms = 25Hz = 主循环 20ms 的 2 倍 */
#define MAX_FRAMES    512    /* 最多 512 帧 → 20.5 秒 */

/* 十字键每帧动几度（按住 = 连续）*/
#define JOG_SLOW      1
#define JOG_FAST      5

/* Flash：芯片最后 2 页（1KB 一页），链接脚本已经把 FLASH 从 64K 让到 62K */
#define FLASH_ADDR    0x0800F800UL
#define FLASH_PAGES   2
#define FLASH_MAGIC   0x54454143UL      /* "TEAC" —— 判断这块 Flash 里有没有我们的数据 */
#define FLASH_HDR     8                 /* magic(4) + 帧数(2) + 保留(2) */

/* ---------------------------------------------------------------------------
 * ★ 编译期守住 Flash 保留区的边界。
 *
 * 保留区是 2 页 = 2048 字节，减去 8 字节头，能放 (2048-8)/3 = 680 帧。
 * 只要有人把 MAX_FRAMES 调过 680，写地址就会越过 0x0800FFFF
 * （F103C8 的 64K 边界），而 HAL_FLASH_Program 只在【运行时】默默失败 ——
 * 车上表现是"按了存，灯闪 1 下说失败"，但根因很难查。
 * 这里直接让它【编译不过】，把问题拦在上位机。
 * --------------------------------------------------------------------------- */
_Static_assert(FLASH_HDR + MAX_FRAMES * JOINT_N <= FLASH_PAGES * 1024,
               "MAX_FRAMES 太大：录制数据会越出 Flash 保留区（2048 字节）");

/* ==================== 录制数据 ==================== */
/* frames[第几帧][第几个关节] = 角度（0~180，一个字节够用）*/
static uint8_t  frames[MAX_FRAMES][JOINT_N];
static uint16_t frame_n;      /* 已经录了多少帧 */
static uint16_t cursor;       /* 回放到第几帧 */

/* ==================== 运行状态 ==================== */
static uint8_t  recording;
static uint8_t  playing;
static uint32_t t_last;       /* 上一帧的时刻（录制/回放共用）*/

/* 通道顺序：和 servo.h 的通道号一一对应 */
static const uint8_t ch_of[JOINT_N] = { CH_ARM, CH_ELBOW, CH_GRIP };

/* 三个关节当前角度。手动点动时改它；回放时它也跟着变，
   这样回放完接着手动操作，不会从旧角度跳过去。 */
static int16_t angle[JOINT_N] = { 90, 90, 90 };

static uint8_t sel = 0;           /* 当前选中的关节下标 */
static int16_t jog = JOG_SLOW;    /* 十字键步长 */

/* ==================== 小工具 ==================== */
static void led(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 闪 n 下，给"操作成功/失败"的反馈 */
static void blink_msg(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        led(1); HAL_Delay(90);
        led(0); HAL_Delay(90);
    }
    HAL_Delay(250);
}

static int16_t clamp180(int16_t d)
{
    if (d < 0)   return 0;
    if (d > 180) return 180;
    return d;
}

/* ---------------------------------------------------------------------------
 * 把三个角度下发给舵机 —— ★ 走【正常接口】：带软限位 + 缓动
 *
 * 为什么要改（原来是 servo_cal_angle() 直写，有三重问题）：
 *   ① servo.h 自己把 servo_cal_angle() 标注为"标定专用、正式跑车不要调"；
 *   ② 它跳过软限位，而 clamp180() 只夹到 0~180 ——
 *      按住十字键 1 秒就能把大臂推到 0°/180° 顶死机械限位
 *      （大舵机堵转发热 / 小舵机崩塑料齿），而且越限值会被录进 Flash 永久回放；
 *   ③ 它会（原来）把 joint[].hard 永久置 1，让软限位彻底失效。
 *
 * 现在改用 arm_set_target()：servo.c 内部按 ARM_MIN/MAX、ELBOW_MIN/MAX 夹，
 * 再由 servo_poll() 按 ARM_SPEED/ELBOW_SPEED/GRIP_SPEED 的"度/秒"缓动过去。
 * 于是速度限幅也真正生效了（原来按住键是 250°/s，是大臂标称 90°/s 的 2.8 倍）。
 *
 * ★ 下发之后要把夹过的值同步回 angle[]，
 *   否则界面上/状态里的角度会和舵机实际目标不一致。
 * ------------------------------------------------------------------------- */
static void apply(void)
{
    for (uint8_t i = 0; i < JOINT_N; i++)
    {
        angle[i] = clamp180(angle[i]);
        arm_set_target(ch_of[i], angle[i]);      /* 软限位在 servo.c 里夹 */
        angle[i] = arm_get_target(ch_of[i]);     /* ★ 把夹过的值同步回来 */
    }
}

/* ==================== Flash：存 ==================== */
/* F103 的 Flash 两个硬规矩：
     ① 只能整页擦（这里 1KB 一页）→ 一次擦 2 页
     ② 只能按"半字"（16 位）写 → 两个字节拼成一个 uint16_t 再写
   所以下面全是半字操作，别改成按字节写。 */
static uint8_t flash_save(void)
{
    uint8_t *p;
    uint16_t nbytes;
    uint8_t ok = 1;

    if (frame_n == 0) return 0;                   /* 没录过，不存 */

    /* ★ 通知底盘"我要写 Flash 了"（会卡 100~200ms），让它们先停车。
       底盘那边定义了 teach_before_save() 就会自动覆盖这个空实现。 */
    teach_before_save();

    nbytes = (uint16_t)(frame_n * JOINT_N);
    p      = (uint8_t *)frames;                   /* 把二维数组当一维字节流看 */

    HAL_FLASH_Unlock();

    /* ① 擦掉 2 页 */
    {
        FLASH_EraseInitTypeDef e;
        uint32_t page_err = 0;

        e.TypeErase    = FLASH_TYPEERASE_PAGES;
        e.PageAddress  = (uint32_t)FLASH_ADDR;
        e.NbPages      = FLASH_PAGES;

        if (HAL_FLASHEx_Erase(&e, &page_err) != HAL_OK) ok = 0;
    }

    /* ② 写头：magic + 帧数 */
    if (ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDR + 0,
                                (uint32_t)(FLASH_MAGIC & 0xFFFFu)) != HAL_OK) ok = 0;
    if (ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDR + 2,
                                (uint32_t)(FLASH_MAGIC >> 16)) != HAL_OK)    ok = 0;
    if (ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDR + 4,
                                (uint32_t)frame_n) != HAL_OK)                ok = 0;
    if (ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDR + 6,
                                0u) != HAL_OK)                               ok = 0;

    /* ③ 写帧数据：两个字节拼成一个半字。
          注意 nbytes = 帧数 × 3，**帧数是奇数时 nbytes 也是奇数**，
          所以按半字循环之后还要单独处理最后一个字节。 */
    for (uint16_t i = 0; ok && (i + 1 < nbytes); i += 2)
    {
        uint16_t hw = (uint16_t)(p[i] | ((uint16_t)p[i + 1] << 8));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              FLASH_ADDR + FLASH_HDR + i, hw) != HAL_OK) ok = 0;
    }
    if (ok && (nbytes & 1u))
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              FLASH_ADDR + FLASH_HDR + (nbytes - 1),
                              (uint32_t)p[nbytes - 1]) != HAL_OK) ok = 0;
    }

    HAL_FLASH_Lock();
    return ok;
}

/* ==================== Flash：读 ==================== */
/* Flash 是内存映射的，直接当指针读就行，不需要"读命令" */
static uint8_t flash_load(void)
{
    const uint32_t *hdr = (const uint32_t *)FLASH_ADDR;
    uint16_t n;
    const uint8_t *p;

    if (hdr[0] != FLASH_MAGIC) return 0;      /* 从没存过 */

    n = *(const uint16_t *)(FLASH_ADDR + 4);
    if (n == 0 || n > MAX_FRAMES) return 0;   /* 头坏了 */

    p = (const uint8_t *)(FLASH_ADDR + FLASH_HDR);

    /* -------------------------------------------------------------------
     * ★ 两遍处理：先【只校验】，全部通过再拷进 frames[]。
     *
     * 原来是一边校验一边写 frames —— 万一第 300 帧发现坏字节就 return 0，
     * 此时 frames 已经被改了前 300 帧，而 frame_n 还是旧值（那句赋值没执行）。
     * 结果按 START 会回放"一半旧数据 + 一半刚读进来的数据"的混合轨迹，
     * 而不是明确地拒绝。
     *
     * 现在：校验不过就【原样保留旧的 frames】，调用方从返回值知道失败。
     * ------------------------------------------------------------------- */
    for (uint32_t i = 0; i < (uint32_t)n * JOINT_N; i++)
    {
        if (p[i] > 180) return 0;             /* 数据坏了，整块不认 */
    }

    for (uint16_t i = 0; i < n; i++)
    {
        for (uint8_t j = 0; j < JOINT_N; j++)
        {
            frames[i][j] = *p++;
        }
    }

    frame_n = n;
    cursor  = 0;                              /* ★ 读回后游标归零，语义清晰 */
    return 1;
}

/* ==================== 采一帧（录制） ==================== */
static void capture_frame(void)
{
    if (frame_n >= MAX_FRAMES) { recording = 0; return; }   /* 录满自动停 */

    for (uint8_t i = 0; i < JOINT_N; i++)
    {
        /* ★ 记"缓动器当前实际所在角度"，不是目标角 ——
           要重放的是实实在在走过的那条轨迹。
           （现在 apply() 走的是带缓动的 arm_set_target()，
             所以这句注释终于是真的了：录下来的确实是实际轨迹。）*/
        frames[frame_n][i] = (uint8_t)clamp180(arm_get_angle(ch_of[i]));
    }
    frame_n++;
}

/* ==================== 放一帧（回放） ==================== */
/* ★ 同样走正常接口：软限位 + 缓动。
   原来用 servo_cal_angle() 是"瞬时跳变"，录下来的轨迹再好也会被跳变毁掉。 */
static void play_frame(uint16_t k)
{
    for (uint8_t i = 0; i < JOINT_N; i++)
    {
        angle[i] = (int16_t)frames[k][i];
        arm_set_target(ch_of[i], angle[i]);
        angle[i] = arm_get_target(ch_of[i]);   /* 同步夹过的值 */
    }
}

/* ===========================================================================
 * ★ 给底盘留的钩子
 * ---------------------------------------------------------------------------
 * 存 Flash 前会被调用一次。默认什么都不做。
 * 底盘那边只要在自己的 .c 里写一个同名函数（不带 __WEAK），
 * 链接时就会用他们的版本，自动在这里停车。
 * =========================================================================== */
__WEAK void teach_before_save(void)
{
}

/* ===========================================================================
 * ★ ST-Link 遥控接口（不需要手柄、不需要任何额外硬件）
 * ---------------------------------------------------------------------------
 * 手柄用不了的时候，怎么驱动这套示教？
 *   用 ST-Link 直接往 RAM 里写命令。CubeProgrammer 的 -w32 可以写内存：
 *
 *       STM32_Programmer_CLI -c "port=SWD mode=hotplug" -w32 <rc_in地址> <命令字>
 *
 *   一个 32 位字 = 一条命令：
 *       bit[7:0]   cmd   命令码
 *       bit[15:8]  p1    参数1
 *       bit[23:16] p2    参数2
 *       bit[31:24] p3    参数3
 *
 *   命令码：
 *       1  SET_ONE    p1=关节(0大臂/1肘/2夹爪), p2=角度0~180
 *       2  SET_ALL    p1,p2,p3 = 大臂/肘/夹爪 三个角度
 *       3  REC_START  开始录制（清零后立刻采第 0 帧）
 *       4  REC_STOP   结束录制
 *       5  PLAY       回放（从第 0 帧开始）
 *       6  STOP       停止录制/回放
 *       7  SAVE       存进 Flash
 *       8  LOAD       从 Flash 读回
 *       9  CENTER     三个关节回中位 90°
 *      10  CLEAR      清空录制（帧数归零）
 *
 *   固件回的状态（只读，我这边用 -r32 读）：
 *       rc_st[0] = (帧数 << 16) | 回放游标
 *       rc_st[1] = (大臂角 << 16) | 肘角
 *       rc_st[2] = (夹爪角 << 16) | 标志位
 *                  标志位: bit0=录制中 bit1=回放中 bit2=手柄在线 bit3=上次存/读成功
 *       rc_cnt   = 已执行命令计数（证明固件真的在处理）
 * =========================================================================== */

volatile uint32_t rc_in  = 0;      /* 我往这里写命令 */
volatile uint32_t rc_st[4];        /* 固件回的状态 */
volatile uint32_t rc_cnt = 0;      /* 已执行命令数 */

#define RC_SET_ONE    1
#define RC_SET_ALL    2
#define RC_REC_START  3
#define RC_REC_STOP   4
#define RC_PLAY       5
#define RC_STOP       6
#define RC_SAVE       7
#define RC_LOAD       8
#define RC_CENTER     9
#define RC_CLEAR     10

static uint8_t rc_last_ok = 0;     /* 上次 存/读 的结果 */

static void rc_poll(void)
{
    uint32_t w = rc_in;
    uint8_t  known = 1;          /* 这条命令是否被识别（决定 rc_cnt 加不加）*/

    if (w != 0)
    {
        uint8_t cmd = (uint8_t)( w        & 0xFFu);
        uint8_t p1  = (uint8_t)((w >>  8) & 0xFFu);
        uint8_t p2  = (uint8_t)((w >> 16) & 0xFFu);
        uint8_t p3  = (uint8_t)((w >> 24) & 0xFFu);

        /* ★ 先把入口清零，再执行命令。
           原来是在 switch【之后】才 rc_in = 0 —— RC_SAVE / RC_LOAD 里
           flash_save() 要阻塞 100~200ms（含整页擦除），这期间主机写进来的
           新命令会被那句 rc_in = 0 直接覆盖丢掉，而且没有任何重试机制
           （对"遥控急停"尤其危险）。
           现在先清空：执行期间到达的命令留在 rc_in 里，下一轮照样处理。 */
        rc_in = 0;

        switch (cmd)
        {
            case RC_SET_ONE:
                playing = 0;   /* ★ 先停回放，否则写下去的角度会在 40ms 内被覆盖 */
                if (p1 < JOINT_N) { angle[p1] = (int16_t)p2; apply(); }
                else              { known = 0; }        /* 关节号越界：不算执行成功 */
                break;

            case RC_SET_ALL:
                playing = 0;   /* ★ 同上：先停回放 */
                angle[0] = (int16_t)p1;
                angle[1] = (int16_t)p2;
                angle[2] = (int16_t)p3;
                apply();
                break;

            case RC_REC_START:
                frame_n   = 0;
                cursor    = 0;
                recording = 1;
                playing   = 0;
                t_last    = HAL_GetTick();
                capture_frame();
                break;

            case RC_REC_STOP:
                recording = 0;
                break;

            case RC_PLAY:
                if (frame_n > 0)
                {
                    playing   = 1;
                    recording = 0;
                    cursor    = 0;
                    t_last    = HAL_GetTick();
                    play_frame(0);
                    cursor = 1;
                }
                break;

            case RC_STOP:
                playing = 0; recording = 0;
                break;

            case RC_SAVE:
                playing = 0; recording = 0;
                rc_last_ok = flash_save();
                break;

            case RC_LOAD:
                playing = 0; recording = 0;
                rc_last_ok = flash_load();
                break;

            case RC_CENTER:
                for (uint8_t i = 0; i < JOINT_N; i++) angle[i] = 90;
                apply();
                break;

            case RC_CLEAR:
                frame_n = 0; cursor = 0;
                break;

            default:
                known = 0;      /* 未知命令码：不计数 */
                break;
        }

        /* ★ 只统计【真正执行了】的命令。
           原来未知命令码也会让 rc_cnt++，而主机正是拿 rc_cnt 当
           "命令已执行"的判据 —— 会被误导。 */
        if (known) rc_cnt++;
    }

    /* 状态一直刷新，随时可读 */
    rc_st[0] = ((uint32_t)frame_n << 16) | (uint32_t)cursor;
    rc_st[1] = ((uint32_t)(uint16_t)angle[0] << 16) | (uint32_t)(uint16_t)angle[1];
    rc_st[2] = ((uint32_t)(uint16_t)angle[2] << 16)
             | (uint32_t)((recording ? 1u : 0u) | (playing ? 2u : 0u)
                        | (ps2_ok ? 4u : 0u) | (rc_last_ok ? 8u : 0u));
}
/* ==================== 初始化（上电调一次） ==================== */
void teach_only_init(void)
{
    arm_init();      /* 舵机：TIM4 三路 50Hz PWM，摆到中位 */
    apply();         /* 内部角度也同步成中位 */

    HAL_Delay(500);  /* 等电源稳一稳 */

    if (flash_load()) blink_msg(2);   /* 闪 2 下 = 读到了上次存的录制 */
}

/* ==================== 每 20ms 调一次 ====================
   前提：调用方刚刚调过 ps2_poll()，ps2_ok / ps2_btn1 / ps2_btn2 是新的。 */
void teach_only_poll(void)
{
    static uint8_t  p1 = 0, p2 = 0;      /* 上一轮按键（边沿检测用）*/
    static uint32_t last = 0;

    uint8_t  b1, b2, r1, r2, moved = 0;

    if (last == 0) last = HAL_GetTick();   /* 第一次调用时对齐节拍 */

    /* ★ ST-Link 遥控：没有手柄也能驱动整套示教（见文件里的 rc_poll 说明）*/
    rc_poll();

    /* ★★ 推进缓动 —— 必须每轮调（50Hz）。
       它是唯一让 joint[].cur 朝 target 移动的地方：少了这一句，
       apply()/play_frame() 设下去的目标永远不会被执行，
       缓动和"度/秒"速度限幅就全部失效
       （原来就是这样 —— 全工程搜 servo_poll 只有定义，没有调用点）。 */
    servo_poll();

    /* ---------------- ① 状态灯 ----------------
       ★ 优先级改成：录制 / 回放 > 手柄掉线 > 待命。
         原来"掉线"优先级最高，结果本模块主打的【无手柄遥控】场景下
         完全看不出到底在不在录/放（灯永远只是慢闪）。
         手柄在线与否可以从 rc_st[2] 的 bit2 读到。 */
    if (recording)       led((uint8_t)((HAL_GetTick() / 500) & 1));   /* 很慢闪 */
    else if (playing)    led((uint8_t)((HAL_GetTick() /  80) & 1));   /* 快闪 */
    else if (!ps2_ok)    led((uint8_t)((HAL_GetTick() / 250) & 1));   /* 掉线：慢闪 */
    else                 led(1);                                      /* 待命：常亮 */

    /* ---------------- ② 按键 ---------------- */
    if (ps2_ok)
    {
        b1 = ps2_btn1;
        b2 = ps2_btn2;
        /* ★ 边沿检测：刚按下 = 现在 & ~上一轮。
           组1 = SELECT/START/十字键；组2 = L1/R1/L2/R2/△○×□ —— 别搞混。 */
        r1 = (uint8_t)(b1 & ~p1);
        r2 = (uint8_t)(b2 & ~p2);

        /* ---- SELECT：开始 / 结束录制 ---- */
        if (r1 & PS2_BTN_SELECT)
        {
            if (recording) { recording = 0; }
            else
            {
                frame_n   = 0;
                cursor    = 0;
                recording = 1;
                playing   = 0;               /* 录制和回放互斥 */
                t_last    = HAL_GetTick();
                capture_frame();             /* 第 0 帧 = 现在的位置 */
            }
        }

        /* ---- START：开始 / 停止回放 ---- */
        if (r1 & PS2_BTN_START)
        {
            if (playing) { playing = 0; }
            else if (frame_n > 0)
            {
                playing   = 1;
                recording = 0;
                cursor    = 0;
                t_last    = HAL_GetTick();
                play_frame(0);
                cursor = 1;
            }
        }

        /* ---- L1 / R1：切换选中关节 ---- */
        if (r2 & PS2_BTN_L1) sel = (uint8_t)((sel + 1) % JOINT_N);
        if (r2 & PS2_BTN_R1) sel = (uint8_t)((sel + JOINT_N - 1) % JOINT_N);

        /* ---- L2 / R2：切换步长 ---- */
        if (r2 & PS2_BTN_L2) jog = (jog == JOG_SLOW) ? JOG_FAST : JOG_SLOW;
        if (r2 & PS2_BTN_R2) jog = (jog == JOG_SLOW) ? JOG_FAST : JOG_SLOW;

        /* ===================================================================
         * ★ 以下是"手动插手"的四项，回放时【全部禁止】。
         *   原来的 if (!playing) 只包住了十字键上/下，
         *   而 △ / ○ / 十字键左·右 在回放中照样生效 ——
         *   回放每 40ms 写回帧值、你每 20ms 写一次目标，两边抢同一个通道，
         *   关节会抖一下，和"回放时不许手动插手"的意图也不一致。
         * =================================================================== */
        if (!playing)
        {
            /* ---- △：全部回中位；○：选中关节回中位 ---- */
            if (r2 & PS2_BTN_TRIANGLE)
            {
                for (uint8_t i = 0; i < JOINT_N; i++) angle[i] = 90;
                moved = 1;
            }
            if (r2 & PS2_BTN_CIRCLE)
            {
                angle[sel] = 90;
                moved = 1;
            }

            /* ---- 十字键 左/右：夹爪 张开 / 夹紧 ---- */
            if (r1 & PS2_BTN_LEFT)  { angle[2] = GRIP_OPEN;  moved = 1; }
            if (r1 & PS2_BTN_RIGHT) { angle[2] = GRIP_CLOSE; moved = 1; }

            /* ---- 机械臂点动：十字键 上/下（按住连续）----
                用 b1（电平）而不是 r1（边沿）：按住就一直动，教动作才顺手 */
            if (b1 & PS2_BTN_UP)   { angle[sel] = (int16_t)(angle[sel] + jog); moved = 1; }
            if (b1 & PS2_BTN_DOWN) { angle[sel] = (int16_t)(angle[sel] - jog); moved = 1; }
        }

        /* ---- □：存进 Flash ---- */
        if (r2 & PS2_BTN_SQUARE)
        {
            playing = 0; recording = 0;
            blink_msg(flash_save() ? 3 : 1);
        }

        /* ---- ×：从 Flash 读回来 ---- */
        if (r2 & PS2_BTN_CROSS)
        {
            playing = 0; recording = 0;
            blink_msg(flash_load() ? 2 : 1);
        }

        if (moved) apply();

        p1 = b1;
        p2 = b2;
    }
    else
    {
        /* ★ 掉线时把"上一轮按键"设成【全部按住】(0xFF)：
           这样重连后的第一帧 r = b & ~p 恒为 0，不会凭空产生边沿。
           原来写 p1 = p2 = 0，会让重连瞬间"正按着的键"被当成刚按下 ——
           可能凭空触发存 Flash（要擦 2 页、阻塞 200ms）或回放。 */
        p1 = 0xFF;
        p2 = 0xFF;
    }

    /* ---------------- ③ 录制 / 回放的定时推进 ----------------
       两边共用同一个 t_last 和 SAMPLE_MS，节奏天然一致。 */
    {
        uint32_t now = HAL_GetTick();

        if ((recording || playing) && (now - t_last >= SAMPLE_MS))
        {
            /* ★ 用 t_last += SAMPLE_MS 累加，而不是 t_last = now：
               后者一旦某一圈超时（写 Flash、手柄读慢了），检查点就滑到下一圈，
               该帧被拉长且【永远补偿不回来】—— "40ms = 正好 2 圈"的前提一破，
               25Hz 采样率就不成立（录 20.5 秒会变长）。
               兜底：落后超过 2 个周期就重新对齐，避免长时间阻塞后连补一大堆帧。 */
            if (now - t_last > (uint32_t)(2 * SAMPLE_MS)) t_last = now;
            else                                          t_last += SAMPLE_MS;

            if (recording) { capture_frame(); }
            else if (playing)
            {
                if (cursor < frame_n) { play_frame(cursor); cursor++; }
                else                    playing = 0;      /* 放完了 */
            }
        }
    }

    /* ---------------- ④ 控节拍：50Hz ----------------
       合并到别人的主循环时，这一句要注释掉（节拍由主循环统一管）。 */
#if !defined(TEACH_ONLY_NO_TICK)
    while (HAL_GetTick() - last < 20) {}
    last = HAL_GetTick();
#endif
}

/* ==================== 独立跑（本工程用这个） ==================== */
void teach_only_main(void)
{
    dwt_init();          /* DWT 微秒延时（PS2 时序要用）*/
    ps2_init();          /* PA4~PA7 + PC13 */

    teach_only_init();   /* 舵机 + Flash 读回 */

    while (1)
    {
        ps2_poll();          /* ★ 手柄统一在这里读一次 */
        teach_only_poll();   /* ★ 机械臂示教 */
        /* 节拍在 teach_only_poll() 里控好了 */
    }
}
