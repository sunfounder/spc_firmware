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

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 240 = 25KHz
// #define _PWMA_PSCR 2           // 预分频 2
// #define _PWMA_PERIOD 240      // PWM周期 (PWMA_ARR自动重装值) 240

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 600 = 10KHz
// #define _PWMA_PSCR 2     // 预分频 2
// #define _PWMA_PERIOD 600 // PWM周期 (PWMA_ARR自动重装值) 600

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 6000 = 1KHz
// #define _PWMA_PSCR 2      // 预分频 2
// #define _PWMA_PERIOD 6000 // PWM周期 (PWMA_ARR自动重装值) 6000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 4 / 6000 = 500Hz
// #define _PWMA_PSCR 4          // 预分频 4
// #define _PWMA_PERIOD 6000      // PWM周期 (PWMA_ARR自动重装值) 6000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 24 / 2,500 = 200Hhz
// #define _PWMA_PSCR 24          // 预分频 24
// #define _PWMA_PERIOD 2500      // PWM周期 (PWMA_ARR自动重装值) 2,500

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 24 / 5,000 = 100Hhz
// #define _PWMA_PSCR 24          // 预分频 24
// #define _PWMA_PERIOD 5000     // PWM周期 (PWMA_ARR自动重装值) 5,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 40MHz / 16 / 25,000 = 100Hhz
#define _PWMA_PSCR 16      // 预分频 24
#define _PWMA_PERIOD 25000 // PWM周期 (PWMA_ARR自动重装值) 25,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 16 / 10,000 = 75Hz
// #define _PWMA_PSCR 16      // 预分频 16
// #define _PWMA_PERIOD 10000 // PWM周期 (PWMA_ARR自动重装值) 10,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 40MHz / 16 / 33,333 = 75Hz
// #define _PWMA_PSCR 16      // 预分频 16
// #define _PWMA_PERIOD 33333 // PWM周期 (PWMA_ARR自动重装值) 33,333

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 12 / 20,000 = 50Hz
// #define _PWMA_PSCR 12      // 预分频 12
// #define _PWMA_PERIOD 20000 // PWM周期 (PWMA_ARR自动重装值) 20,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 24 / 25,000 = 20Hz
// #define _PWMA_PSCR 24           // 预分频 24
// #define _PWMA_PERIOD 25000      // PWM周期 (PWMA_ARR自动重装值) 20,000

void FanInit();
void FanSetSpeed(u8 speed);

#endif
