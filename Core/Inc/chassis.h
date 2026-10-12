/*
 * chassis.h —— 底盘运动层（麦轮全向解算）
 *
 * 麦克纳姆轮是"完整约束"（holonomic）底盘，3 个自由度可以同时任意组合：
 *     vx     前进 / 后退
 *     vy     左 / 右横移
 *     omega  原地自转
 * 所以它能斜着走、能横着平移、能边走边转，不是只有 8 个方向。
 *
 * 用法： chassis_init();
 *        chassis_drive(vx, vy, omega);     // 三个都是 -800 ~ +800
 *
 * 坐标约定（人站在车后面往前看）：
 *     vx    > 0  前进              vx    < 0  后退
 *     vy    > 0  向右平移          vy    < 0  向左平移
 *     omega > 0  左转（从上往下看逆时针）    omega < 0  右转
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* 底盘速度上限。比 MOTOR_SPEED_MAX 小一点，三个自由度叠加时才有余量，
   不会一推杆就顶到满速把电流拉爆。 */
#define CHASSIS_SPEED_MAX   800

/* 横移方向开关。
 * 麦轮的辊子有左旋 / 右旋两种装法，"vy 为正到底是不是往右走"由实物决定，
 * 光看代码定不了。实测时如果发现"右推摇杆车往左走"，把这里改成 (-1) 即可。 */
#define CHASSIS_STRAFE_SIGN   (+1)

void chassis_init(void);                                  /* 初始化电机 PWM */
void chassis_stop(void);                                  /* 四轮滑行停车 */

/* 全向指令：前进 + 横移 + 自转，三个分量同时生效 */
void chassis_drive(int16_t vx, int16_t vy, int16_t omega);

/* 下面几个是给自检和上层调用方便写的糖 */
void chassis_forward(int16_t speed);
void chassis_backward(int16_t speed);
void chassis_turn_left(int16_t speed);
void chassis_turn_right(int16_t speed);
void chassis_strafe_left(int16_t speed);
void chassis_strafe_right(int16_t speed);

/* 单独转某一个轮子（自检用），wheel = MOTOR_FL ~ MOTOR_RR */
void chassis_wheel(uint8_t wheel, int16_t speed);

#endif /* CHASSIS_H */
