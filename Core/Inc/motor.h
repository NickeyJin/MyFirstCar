/*
 * motor.h —— 单轮电机硬件层（DRV8833 双 PWM 驱动）
 *
 * 一片 DRV8833 有两个 H 桥，每个 H 桥用 IN1 / IN2 两根脚：
 *     IN1=0    IN2=0     高阻滑行（停）
 *     IN1=PWM  IN2=0     正转，占空比 = 转速
 *     IN1=0    IN2=PWM   反转，占空比 = 转速
 *     IN1=1    IN2=1     刹车（急停）
 *
 * 所以一个轮子要吃掉 2 路 PWM，4 个轮子 = 8 路：
 *     1 左前 FL   PA0 / PA1    TIM2_CH1 / CH2
 *     2 右前 FR   PA2 / PA3    TIM2_CH3 / CH4
 *     3 左后 RL   PB0 / PB1    TIM3_CH3 / CH4
 *     4 右后 RR   PA8 / PA11   TIM1_CH1 / CH4
 *
 * 用法： motor_init();  然后  motor_set(轮号, -1000 ~ +1000)
 *        speed > 0 表示"该轮按车前进的方向转"（接反的轮子在 motor_map 里翻 invert）
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 速度满量程：motor_set() 的取值范围就是 -MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX */
#define MOTOR_SPEED_MAX   1000

/* 三个电机定时器都是 PSC=71 / ARR=999，所以 PWM 频率 = 72MHz / 72 / 1000 = 1kHz
 *
 * 为什么用 1kHz 而不是教科书上的 20kHz：
 *   实测（2026-10-11，闪灯分段排查）同一套硬件下
 *     20kHz + 40% 占空比 → 轮子完全不转
 *     20kHz + 80% 占空比 → 转，但明显比下面慢
 *     1kHz  + 80% 占空比 → 转得最快
 *   说明驱动板的开关沿有固定损耗，频率越高、每个周期的"有效导通时间"被吃得越多；
 *   1kHz 时损耗占比可以忽略，指令占空比 ≈ 实际占空比。
 *   代价是 1kHz 落在人耳可听范围，会有轻微嗡嗡声；嫌吵可以试 2~4kHz（改 PSC/ARR 即可）。
 *
 * 这里直接用 ARR 当满占空比的基准，避免改 .ioc 之后忘了同步这个宏。 */
#define MOTOR_PWM_ARR     999

/* 轮位编号（下标从 1 开始，和硬件丝印 FL/FR/RL/RR 一一对应） */
#define MOTOR_FL   1    /* 左前 */
#define MOTOR_FR   2    /* 右前 */
#define MOTOR_RL   3    /* 左后 */
#define MOTOR_RR   4    /* 右后 */

void motor_init(void);                          /* 启动 8 路 PWM 并让四轮停住 */
void motor_set(uint8_t wheel, int16_t speed);   /* 轮号 1~4，速度 -1000 ~ +1000 */
void motor_stop_all(void);                      /* 四轮高阻滑行 */
void motor_brake_all(void);                     /* 四轮刹车（急停） */

#endif /* MOTOR_H */
