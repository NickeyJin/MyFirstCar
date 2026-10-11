/*
ps2.c —— PS2 无线手柄驱动的实现
    ps2_init()   配置 PA4~PA7 和状态灯 PC13
    ps2_xfer()   收发 1 个字节
    ps2_read()   收取 9 个字节
    ps2_poll()   调 ps2_read() 并解析成 ps2.h 里那几个变量
*/
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>
#include "dwt_delay.h"
#include "ps2.h"

uint8_t ps2_ok;                  /* 本轮数据有效吗 */
uint8_t ps2_btn1;                /* 按键组1 */
uint8_t ps2_btn2;                /* 按键组2 */
uint8_t ps2_lx, ps2_ly;          /* 左摇杆 */
uint8_t ps2_rx, ps2_ry;          /* 右摇杆 */

static uint8_t ps2_buf[9];       /* 接收字节的缓冲区 */
static const uint8_t ps2_cmd[9] =    /* 我们要发出去的 9 个字节 */
{
    0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void ps2_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Pin   = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin  = GPIO_PIN_4;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    /* PC13 状态灯：测试用灯！ */
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &g);

    /* 空闲状态：这几个引脚一定要给高电平，不然会有bug！ */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
}

static uint8_t ps2_xfer(uint8_t cmd)
{
    uint8_t res = 0;
    uint8_t ref;

    for (ref = 0x01; ref != 0; ref <<= 1)      
    {

        if (cmd & ref) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        else           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        delay_us(16);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)  res |= ref;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        delay_us(16);

    }

    return res;
}

static void ps2_read(void)
{
    uint8_t       *dst = ps2_buf;
    const uint8_t *src = ps2_cmd;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < 9; i++) *dst++ = ps2_xfer(*src++);     /*你知道a++与++a的区别吗，嘻嘻，指针真是太简单啦！*/
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}

void ps2_poll(void)
{
    ps2_read();

    const uint8_t *b = ps2_buf;
    uint8_t id = b[1];

    /* 0x41=数字  0x73=模拟(红)  0x53=模拟(绿)  0x79=真的用的时候 */
    ps2_ok   = (id == 0x41) || (id == 0x73) || (id == 0x53) || (id == 0x79);

    /* 取反更符合判断，按下的时候反而是0，取个反正好 */
    ps2_btn1 = ~b[3]; ps2_btn2 = ~b[4];
    /* 右摇杆和左摇杆的数据 */
    ps2_rx = b[5]; ps2_ry = b[6];
    ps2_lx = b[7]; ps2_ly = b[8];
}