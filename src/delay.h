#ifndef __DELAY_H
#define __DELAY_H

#include "types.h"

/** 定义指令集，不同指令集的机器周期不同, 指令的耗时也不同，这里仅考虑 循环递减的耗时，实现的延时会有误差
 *  _STC_Y1 ： stc89/stc90 , 机器周期（12T）
 * _STC_Y6: stc8F/stc8A/stc8G/stc8H, 机器周期（1T）
 * _STC_32: 速度约为普通8051的 80倍
 */
// #define _STC_Y1_12MHz
// #define _STC_Y6_12MHz
// #define _STC32_12MHz
#define _STC32_35MHz
// #define _STC32_40MHz

void delayMs(u32 ms);
void delayUs(u32 us);
void delayS(u32 s);

#endif
