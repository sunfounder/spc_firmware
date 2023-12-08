#ifndef __FAN_H
#define __FAN_H

#include "stc32g.h"
#include "types.h"

/*---------------------------------------------------------------
    fan: (PWMA)
        FAN -> PWM4P_2 -> P2.6
        (注意不是默认的PWM引脚，需要使用 PWMA_PS 寄存器 选择引脚)
---------------------------------------------------------------*/
#define FAN P26
#define FAN_USE_PWM 1

#define MAIN_Fosc 35000000L // 定义主时钟 35MHz

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 35MHz / 16 / 21,875 = 100Hhz
#define _PWMA_PSCR 16      // 预分频 24
#define _PWMA_PERIOD 21875 // PWM周期 (PWMA_ARR自动重装值) 21,875

void FanInit();
void FanSetSpeed(u8 speed);

#endif
