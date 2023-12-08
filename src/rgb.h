#ifndef __RGB_H
#define __RGB_H

#include "stc32g.h"
#include "types.h"

/*---------------------------------------------------------------
    rgb: (PWMB)
        LED_R -> PWM5 -> P2.0
        LED_G -> PWM6 -> P2.1
        LED_B -> PWM7 -> P2.2
---------------------------------------------------------------*/
#define LED_R P20
#define LED_G P21
#define LED_B P22

#define LED_USE_PWM 1
#define MAIN_Fosc 35000000L // 定义主时钟 35MHz

#define _PWMB_PSCR 4     // 预分频 4
#define _PWMB_PERIOD 875 // PWM周期 (PWMB_ARR自动重装值) 875

void rgbInit();
void rgbWrite(u8 r, u8 g, u8 b);
void rgbClose();

#endif
