#ifndef __RGB_H
#define __RGB_H

#include "stc32g.h"
#include "types.h"

/*---------------------------------------------------------------
    rgb: (PWMB)
        LED_B -> PWM5 -> P2.0
        LED_G -> PWM6 -> P2.1
        LED_R -> PWM7 -> P2.2
---------------------------------------------------------------*/
#define LED_B P20
#define LED_G P21
#define LED_R P22

#define LED_USE_PWM 1
#define MAIN_Fosc 12000000L // 定义主时钟 12MHz

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 6000 = 1KHz
// #define _PWMB_PSCR 2      // 预分频 2
// #define _PWMB_PERIOD 6000 // PWM周期 (PWMB_ARR自动重装值) 6000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 600 = 10KHz
#define _PWMB_PSCR 2     // 预分频 2
#define _PWMB_PERIOD 600 // PWM周期 (PWMA_ARR自动重装值) 600

void rgbInit();
void rgbWrite(u8 r, u8 g, u8 b);
void rgbClose();

#endif
