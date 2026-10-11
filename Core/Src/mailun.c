/*
mailun.c —— 麦轮逆运动学
    mecanum_drive() 函数将底盘的三轴速度分配到四个麦轮上。
        vx ：前后方向速度，正的是前进
        vy ：左右方向速度，正的是右移
        omega ：自转速度，正的是逆时针方向（从上往下看）
*/

#include "main.h"
#include "mailun.h"
#include "motor.h"

#define OMEGA_GAIN  1 /* 自转增益，数字越大转的越快 */

void mecanum_drive(int16_t vx, int16_t vy, int16_t omega)
{
    int32_t w = (int32_t)omega * OMEGA_GAIN;
    int32_t v[5]; /* v[1]=FL v[2]=FR v[3]=RL v[4]=RR，依旧不用v[0] */

    /*这一部分了解了一下，本来以为是矢量相加，看了看原理和数学推导，想出麦轮的人绝对是天才吧，精妙的数值设计！*/
    v[1] = (int32_t)vx - vy - w;
    v[2] = (int32_t)vx + vy + w;
    v[3] = (int32_t)vx + vy - w;
    v[4] = (int32_t)vx - vy + w;

    /* 暑假学习了一下transformer架构，觉得归一化真的是很正确的想法，虽然会让车速慢一点，但是保证不会有bug！ */
    int32_t maxabs = 0;
    for (uint8_t i = 1; i <= 4; i++)
    {
        int32_t t = v[i];
        if (t < 0) t = -t;
        if (t > maxabs) maxabs = t;
    }
    if (maxabs > MOTOR_SPEED_MAX) for (uint8_t i = 1; i <= 4; i++) v[i] = v[i] * MOTOR_SPEED_MAX / maxabs;
    for (uint8_t i = 1; i <= 4; i++) motor_set(i, (int16_t)v[i]);
    /* OIer 们的智慧也是无穷的，压缩代码行神力！ */
}