/*
mailun.h —— 麦轮逆运动学
*/
#ifndef MAILUN_H
#define MAILUN_H

#include <stdint.h>

void mecanum_drive(int16_t vx, int16_t vy, int16_t omega);

#endif // MAILUN_H