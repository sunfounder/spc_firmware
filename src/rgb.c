#include "rgb.h"

/*---------------------------------------------------------------
    rgb: (PWMB)
        LED_B -> PWM5 -> P2.0
        LED_G -> PWM6 -> P2.1
        LED_R -> PWM7 -> P2.2
---------------------------------------------------------------*/
void rgbInit()
{

    // P2M0 |= 0x07;
    // P2M1 &= ~0x07; // 设置P2.0, P2.1, P2.2为推挽输出模式
    P2M0 &= ~0x07;
    P2M1 &= ~0x07; // 设置P2.0, P2.1, P2.2为 准双向口

#if LED_USE_PWM
    // PWMB (PWM5, PWM6, PWM7,)
    P_SW2 |= 0x80; // 使能扩展寄存器(XFR)访问

    // PWMB_PS = 0x00;                             // 高级 PWM 选择寄存器 PWMB_PS, 设置pwm输出引脚

    PWMB_CCER1 = 0x00; // 写CCMRx前必须先清零CCERx关闭通道
    PWMB_CCER2 = 0x00; // 写CCMRx前必须先清零CCERx关闭通道

    PWMB_CCMR1 = 0x68; // 设置 PWM5 输出模式 1, 使能预装载功能
    PWMB_CCMR2 = 0x68; // 设置 PWM6 输出模式 1, 使能预装载功能
    PWMB_CCMR3 = 0x68; // 设置 PWM7 输出模式 1, 使能预装载功能

    // PWMB_CCER1 = 0x33; // 使能PWM5, PWM6通道, 设置极性为 低电平有效
    // PWMB_CCER2 = 0x03; // 使能PWM7通道, 设置极性为 低电平有效

    PWMB_CCER1 = 0x11; // 使能PWM5, PWM6通道, 设置极性为 低电平有效
    PWMB_CCER2 = 0x01; // 使能PWM7通道, 设置极性为 低电平有效

    PWMB_PSCRH = (u8)((_PWMB_PSCR - 1) >> 8); // 设置PWMB预分频
    PWMB_PSCRL = (u8)(_PWMB_PSCR - 1);
    PWMB_ARRH = (u8)((_PWMB_PERIOD - 1) >> 8); // 设置周期时间， PWMB_ARR自动重装值
    PWMB_ARRL = (u8)(_PWMB_PERIOD - 1);

    PWMB_CCR5H = (u8)(0 >> 8); // 设置PWM5初始占空比
    PWMB_CCR5L = (u8)(0);
    PWMB_CCR6H = (u8)(0 >> 8); // 设置PWM6初始占空比
    PWMB_CCR6L = (u8)(0);
    PWMB_CCR7H = (u8)(0 >> 8); // 设置PWM7初始占空比
    PWMB_CCR7L = (u8)(0);

    PWMB_ENO |= 0x01; // 使能 PWM5 输出
    PWMB_ENO |= 0x04; // 使能 PWM6 输出
    PWMB_ENO |= 0x10; // 使能 PWM7 输出

    PWMB_BKR = 0x80; // 使能PWMB主输出
    PWMB_CR1 = 0x01; // 使能PWMB计数器，开始计时, (00 边沿对齐模式)

#endif
}

/*
r, 0 ~ 255
g, 0 ~ 255
b, 0 ~ 255
*/
void rgbWrite(u8 r, u8 g, u8 b)
{
#if LED_USE_PWM
    u16 ccr_r = 0;
    u16 ccr_g = 0;
    u16 ccr_b = 0;

    ccr_r = ((u32)_PWMB_PERIOD * r / 255);
    ccr_g = ((u32)_PWMB_PERIOD * g / 255);
    ccr_b = ((u32)_PWMB_PERIOD * b / 255);

    P_SW2 |= 0x80; // 使能扩展寄存器(XFR)访问
    // PWMB_CCR5H = (u8)(ccr_b >> 8); // 设置PWM5占空比
    // PWMB_CCR5L = (u8)(ccr_b);
    // PWMB_CCR6H = (u8)(ccr_g >> 8); // 设置PWM6占空比
    // PWMB_CCR6L = (u8)(ccr_g);
    // PWMB_CCR7H = (u8)(ccr_r >> 8); // 设置PWM7占空比
    // PWMB_CCR7L = (u8)(ccr_r);

    PWMB_CCR5H = (u8)(ccr_r >> 8); // 设置PWM5占空比
    PWMB_CCR5L = (u8)(ccr_r);
    PWMB_CCR6H = (u8)(ccr_g >> 8); // 设置PWM6占空比
    PWMB_CCR6L = (u8)(ccr_g);
    PWMB_CCR7H = (u8)(ccr_b >> 8); // 设置PWM7占空比
    PWMB_CCR7L = (u8)(ccr_b);

#else
    if (r != 0)
        r = 1;
    if (g != 0)
        g = 1;
    if (b != 0)
        b = 1;
    LED_R = r;
    LED_G = g;
    LED_B = b;
#endif
}

void rgbClose()
{
#if LED_USE_PWM
    P_SW2 |= 0x80;             // 使能扩展寄存器(XFR)访问
    PWMB_CCR5H = (u8)(0 >> 8); // 设置PWM5 占空比 为 0
    PWMB_CCR5L = (u8)(0);
    PWMB_CCR6H = (u8)(0 >> 8); // 设置PWM6 占空比 为 0
    PWMB_CCR6L = (u8)(0);
    PWMB_CCR7H = (u8)(0 >> 8); // 设置PWM7 占空比 为 0
    PWMB_CCR7L = (u8)(0);
#else
    LED_R = 0;
    LED_G = 0;
    LED_B = 0;
#endif
}
