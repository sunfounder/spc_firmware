#include "rtc.h"

RTC_ConfigTypeDef rtcConfig = {
    21, // Y:2021 // NIYEAR, RTC 年 设置 0 ~ 99
    12, // M:12 // NIMONTH, RTC 月 设置 1 ~ 12
    31, // D:31 // NIDAY, RTC 日 设置 1 ~ 31
    23, // H:23 // NIHOUR, RTC 时 设置 0 ~ 23
    59, // M:59 // NIMIN, RTC 分 设置 0 ~ 59
    50, // S:50 // NISEC, RTC 秒 设置 0 ~ 59
    0,  // S/128:0 // NISSEC, RTC S/128 设置 0 ~ 127
};

void RTC_Init(void)
{
    // --- 选择内部低速 IRC ---
    // IRC32KCR = 0x80; // 启动内部低速 IRC振荡器
    // while (!(IRC32KCR & 0x01))
    //     ; // 等待时钟稳定
    // RTCCFG |= 0x02; // 选择内部低速 IRC作为 RTC时钟源

    // --- 选择外部 32K ---
    X32KCR = 0xc0; // 启动外部 32K 晶振
    while (!(X32KCR & 0x01))
        ;            // 等待时钟稳定
    RTCCFG &= ~0x02; // 选择外部 32K 作为 RTC时钟源

    // ConfigRTC(&rtcConfig);

    // INIYEAR = 21;
    // INIMONTH = 12;
    // INIDAY = 31;
    // INIHOUR = 23;
    // INIMIN = 59;
    // INISEC = 59;
    // INISSEC = 0;

    // RTCCFG |= 0x01; // 触发 RTC寄存器初始化

    // RTCCR = 0x01;         // RTC使能
    // while (RTCCFG & 0x01) // 等待初始化完成,需要在 "RTC使能 "之后判断.
    //     ;

    // RTCIF = 0; // 清中断标志
    // // RTCIEN = 0x08; // 使能 RTC秒中断

    RTCCR = 0x01; // RTC使能
}

void ConfigRTC(RTC_ConfigTypeDef *RTCx)
{
    if (RTCx->Year > 99)
        RTCx->Year = 23;
    if (RTCx->Month > 12)
        RTCx->Month = 12;
    if (RTCx->Day > 31)
        RTCx->Day = 31;
    if (RTCx->Hour > 23)
        RTCx->Hour = 23;
    if (RTCx->Min > 59)
        RTCx->Min = 59;
    if (RTCx->Sec > 59)
        RTCx->Sec = 50;
    if (RTCx->Ssec > 127)
        RTCx->Ssec = 0;

    INIYEAR = RTCx->Year;
    INIMONTH = RTCx->Month;
    INIDAY = RTCx->Day;
    INIHOUR = RTCx->Hour;
    INIMIN = RTCx->Min;
    INISEC = RTCx->Sec;
    INISSEC = RTCx->Ssec;

    RTCCFG |= 0x01;       // 触发 RTC寄存器初始化
    RTCCR = 0x01;         // RTC使能
    while (RTCCFG & 0x01) // 等待初始化完成,需要在 "RTC使能 "之后判断.
        ;
    /** 设置 RTC时间需要 32768Hz 的 1 个周期时间,
    大约 30.5us. 由于同步,所以实际等待时间是 0 ~30.5us.
    如果不等待设置完成就睡眠,则 RTC会由于设置没
    完成,停止计数,唤醒后才继续完成设置并继续计数. */
}
