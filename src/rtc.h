#ifndef _RTC_H
#define _RTC_H

#include "stc32g.h"
#include "types.h"

/** 时间跨度：支持 2000 年~2099 年，并自动判断闰年 */
/** RTC 相关的寄存器
RTCCR
RTCCFG
RTCIEN
RTCIF

// --- 闹钟 ---
ALAHOUR
ALAMIN
ALASEC
ALASSEC

// --- 初始化 ---
INIYEAR
INIMONTH
INIDAY
INIHOUR
INIMIN
INISEC
INISSEC // 1/128 秒

// --- 计数值 ---
YEAR
MONTH
DAY
HOUR
MIN
SEC
SSEC // 1/128 秒
*/

typedef struct
{
    u8 Year;  // RTC 年, 00~99, 对应2000~2099年
    u8 Month; // RTC 月, 01~12
    u8 Day;   // RTC 日, 01~31
    u8 Hour;  // RTC 时, 00~23
    u8 Min;   // RTC 分, 00~59
    u8 Sec;   // RTC 秒, 00~59
    u8 Ssec;  // RTC 1/128秒, 00~127
} RTC_ConfigTypeDef;

void RTC_Init(void);
void ConfigRTC(RTC_ConfigTypeDef *RTCx);

#endif
