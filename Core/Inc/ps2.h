/* 
ps2.h —— PS2 无线手柄
作用：让 STM32 能够读取 PS2 无线手柄的数据。
用法：先ps2_init();  然后再在循环里 ps2_poll();
*/
#ifndef PS2_H
#define PS2_H

#include <stdint.h>

extern uint8_t ps2_ok;              /* 判断数据是否有效 */
extern uint8_t ps2_btn1;            /* 按键组1（按下 = 1）*/
extern uint8_t ps2_btn2;            /* 按键组2（按下 = 1）*/
extern uint8_t ps2_lx, ps2_ly;      /* 左摇杆的方向*/
extern uint8_t ps2_rx, ps2_ry;      /* 右摇杆的方向*/

/* ---- 按键组1的8个位，从淘宝网上得到的参考文档里得到---- */
#define PS2_BTN_SELECT  0x01
#define PS2_BTN_L3      0x02
#define PS2_BTN_R3      0x04
#define PS2_BTN_START   0x08
#define PS2_BTN_UP      0x10
#define PS2_BTN_RIGHT   0x20
#define PS2_BTN_DOWN    0x40
#define PS2_BTN_LEFT    0x80

/* ---- 按键组2的8个位，从淘宝网上得到的参考文档里得到---- */
#define PS2_BTN_L2        0x01
#define PS2_BTN_R2        0x02
#define PS2_BTN_L1        0x04
#define PS2_BTN_R1        0x08
#define PS2_BTN_TRIANGLE  0x10   /* 三角 */
#define PS2_BTN_CIRCLE    0x20   /* 圆圈 */
#define PS2_BTN_CROSS     0x40   /* 叉叉 */
#define PS2_BTN_SQUARE    0x80   /* 方块 */

void ps2_init(void);
void ps2_poll(void);

#endif /* PS2_H */