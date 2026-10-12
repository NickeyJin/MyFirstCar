/*
 * ps2.c —— PS2 无线手柄驱动（软件模拟 PS2 时序）
 *
 * 一帧 = 9 个字节，主机先发后收（全双工）：
 *     b[0] 0x01 起始   b[1] 手柄 ID   b[2] 0x5A
 *     b[3] 按键组1（按下 = 0，读回来取反）   b[4] 按键组2
 *     b[5] 右摇杆左右  b[6] 右摇杆上下  b[7] 左摇杆左右  b[8] 左摇杆上下
 *     摇杆 0x00 ~ 0xFF，中位 0x80，0x00 在左/上，0xFF 在右/下
 *
 * 时序（和厂商参考代码 ver. XTARK 一致，位序是低位在前 -> 这里用的是 0x01 左移）：
 *     每来一位：先把 CMD 摆好 -> CLK 拉低 -> 等 16µs -> 采样 DATA -> CLK 拉高 -> 等 16µs
 *     整帧期间 ATT/CS 保持低电平。
 */
#include "main.h"
#include "ps2.h"
#include "dwt_delay.h"

/* PS2 手册给的时钟周期是 12~16µs，取 16µs 最稳（≈31kHz） */
#define PS2_BIT_US      16

/* 一帧读不到就重试几轮，无线接收器经常要连着读几次才吐数据 */
#define PS2_READ_RETRY  4

#define PS2_PORT_DATA   GPIOA
#define PS2_PIN_DATA    GPIO_PIN_4     /* 输入：手柄 -> MCU */
#define PS2_PIN_CMD     GPIO_PIN_5     /* 输出：MCU  -> 手柄 */
#define PS2_PIN_ATT     GPIO_PIN_6     /* 输出：片选，低有效 */
#define PS2_PIN_CLK     GPIO_PIN_7     /* 输出：时钟 */

uint8_t ps2_ok;                  /* 这一帧数据能不能用 */
uint8_t ps2_btn1;                /* 按键组1（已取反：按下 = 1） */
uint8_t ps2_btn2;                /* 按键组2（已取反：按下 = 1） */
uint8_t ps2_lx, ps2_ly;          /* 左摇杆 左右 / 上下 */
uint8_t ps2_rx, ps2_ry;          /* 右摇杆 左右 / 上下 */

static uint8_t ps2_buf[9];       /* 收回来的 9 个字节 */

/* 主机要发出去的 9 个字节：0x01 = 起始，0x42 = 请求数据 */
static const uint8_t ps2_cmd[9] =
{
    0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void ps2_axes_to_center(void)
{
    ps2_btn1 = 0;
    ps2_btn2 = 0;
    ps2_lx = ps2_ly = PS2_STICK_CENTER;
    ps2_rx = ps2_ry = PS2_STICK_CENTER;
}

void ps2_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* CMD / ATT / CLK：推挽输出 */
    g.Pin   = PS2_PIN_CMD | PS2_PIN_ATT | PS2_PIN_CLK;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_PORT_DATA, &g);

    /* DATA：输入 + 内部上拉（接收器是开漏输出，必须有上拉） */
    g.Pin  = PS2_PIN_DATA;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PS2_PORT_DATA, &g);

    /* PC13 状态灯：低电平点亮，只能点灯，不能带负载 */
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);

    /* 空闲态：ATT 高、CLK 高、CMD 高 —— 这三个脚悬着不拉高会出一堆怪问题 */
    HAL_GPIO_WritePin(PS2_PORT_DATA, PS2_PIN_ATT | PS2_PIN_CLK | PS2_PIN_CMD, GPIO_PIN_SET);

    ps2_axes_to_center();
    ps2_ok = 0;
}

/* 收发 1 个字节，返回同时读回来的 1 个字节 */
static uint8_t ps2_xfer(uint8_t cmd)
{
    uint8_t res = 0;

    for (uint8_t ref = 0x01; ref != 0; ref <<= 1)
    {
        HAL_GPIO_WritePin(PS2_PORT_DATA, PS2_PIN_CMD,
                          (cmd & ref) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(PS2_PORT_DATA, PS2_PIN_CLK, GPIO_PIN_RESET);
        delay_us(PS2_BIT_US);

        if (HAL_GPIO_ReadPin(PS2_PORT_DATA, PS2_PIN_DATA) == GPIO_PIN_SET)
        {
            res |= ref;     /* 位序低位在前，和发送端保持一致 */
        }

        HAL_GPIO_WritePin(PS2_PORT_DATA, PS2_PIN_CLK, GPIO_PIN_SET);
        delay_us(PS2_BIT_US);
    }

    return res;
}

/* 拉低 ATT，整帧 9 字节收发完再放开 */
static void ps2_read_frame(void)
{
    HAL_GPIO_WritePin(PS2_PORT_DATA, PS2_PIN_ATT, GPIO_PIN_RESET);

    for (uint8_t i = 0; i < 9; i++)
    {
        ps2_buf[i] = ps2_xfer(ps2_cmd[i]);
    }

    HAL_GPIO_WritePin(PS2_PORT_DATA, PS2_PIN_ATT, GPIO_PIN_SET);
}

/* 手柄 ID：0x41 数字模式，0x73 / 0x53 模拟模式，0x79 部分副厂手柄 */
static uint8_t ps2_id_valid(uint8_t id)
{
    return (id == 0x41) || (id == 0x73) || (id == 0x53) || (id == 0x79);
}

void ps2_poll(void)
{
    uint8_t id = 0;

    for (uint8_t i = 0; i < PS2_READ_RETRY; i++)
    {
        ps2_read_frame();
        id = ps2_buf[1];
        if (ps2_id_valid(id)) break;
    }

    if (!ps2_id_valid(id))
    {
        /* 接收器没插 / 没对码 / 跑太远：整组退回中立，
           上层看到 ps2_ok == 0 就会停车，绝不会拿脏数据乱冲 */
        ps2_ok = 0;
        ps2_axes_to_center();
        return;
    }

    ps2_ok = 1;

    /* 数据字节是"按下 = 0"，取反之后用起来更顺手 */
    ps2_btn1 = (uint8_t)(~ps2_buf[3]);
    ps2_btn2 = (uint8_t)(~ps2_buf[4]);

    if (id == 0x41)
    {
        /* 数字模式：第 5~8 字节不是摇杆值（只会是 0x00 / 0xFF），
           硬当中位算，不然车会自己冲出去 */
        ps2_lx = ps2_ly = PS2_STICK_CENTER;
        ps2_rx = ps2_ry = PS2_STICK_CENTER;
        return;
    }

    ps2_rx = ps2_buf[5];
    ps2_ry = ps2_buf[6];
    ps2_lx = ps2_buf[7];
    ps2_ly = ps2_buf[8];
}
