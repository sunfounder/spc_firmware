/*============================================================================

芯片：STC32G8K64
主频选择： 40MHZ

引脚连接:
    output:
        PWR_CTL -> P3.2
        USB_EN -> P3.5
        DC_EN  -> P3.4
    input:
        PWR_BTN -> P3.3
        CHG -> P0.1
        POWER_SOURCE -> P1.5
        RPI_STATE (pi gpio 26)-> P2.7
        ALWAYS_ON -> P2.3
        BD_0 -> P41
        BD_1 -> P42
        BD_2 -> P43
    dac:
        DAC -> PWM8_3 -> P0.3
    adc:
        VBUS_3V3 -> ADC6 -> P1.6
        BT_LV_3V3 -> ADC7 -> P1.7
        OUTPUT_3V3 -> ADC4 ->P1.4
        BATTERY_CURRENT -> ADC3 ->P1.3
        OUTPUT_CURRENT -> ADC2 ->P1.2
    i2c:
        SDA -> P2.4
        SCL -> P2.5
    rgb: (PWMB)
        LED_B -> PWM5 -> P2.0
        LED_G -> PWM6 -> P2.1
        LED_R -> PWM7 -> P2.2
    fan: (PWMA)
        FAN -> PWM4P_2 -> P2.6
    uart:
        TXD -> P3.1
        RXD -> P3.0

==============================================================================*/
#include "stc32g.h"
#include "intrins.h"

#include "types.h"
#include "delay.h"
#include "rgb.h"
#include "fan.h"
#include "power.h"
#include "rtc.h"

// VERSION INFO
// =================================================================
#define VERSION "0.0.1"

// 产品ID
// =================================================================
/**
   BD2 BD1 BD0    BOARD
    0   0   0    UPS Hat
    0   0   1    Pironman Hat
    -   -   -
*/
#define BD_2 P43
#define BD_1 P42
#define BD_0 P41

u8 edata boardID = 0;

u8 getBoardID()
{
    u8 id = 0;

    P4M0 &= ~0x02;
    P4M1 |= 0x02; // P41 高阻输入
    P3M0 &= ~0x80;
    P3M1 |= 0x80; // P37 高阻输入
    P3M0 &= ~0x40;
    P3M1 |= 0x40; // P36 高阻输入

    if (BD_0)
        id = 0x01;
    else
        id = 0;
    if (BD_1)
        id |= 0x02;
    if (BD_2)
        id |= 0x04;

    return id;
}

// 中断优先级 （0~3级，数值越大，等级越高）
// =================================================================
#define TIME0_INTERRUPT_PRIORITY 2 // PT0H,PT0 (IPH ^ 1, IP ^ 1)
#define TIME1_INTERRUPT_PRIORITY 1 // PT1H,PT1 (IPH ^ 3, IP ^ 3)
#define I2C_INTERRUPT_PRIORITY 0   // PI2CH,PI2C (IP2H ^ 6, IP2 ^ 6)
#define IO_INTERRUPT_PRIORITY 3    // IO 中断优先级

// 函数声明
// =================================================================
void PowerSaveMode();

// 全局变量
// =================================================================
// ---------------data----------------------
u8 edata dataBuffer[] = {
    0, //  board id
    1, //  vccVoltage 内部参考电压高位 mV
    2, //  vccVoltage 内部参考电压低位 mV

    3, // usb_voltage 高位 mV
    4, // usb_voltage 低位 mV

    5, // usb_current 高位 mA
    6, // usb_current 低位 mA

    7, // output_voltage 高位 mV
    8, // output_voltage 低位 mV

    9,  // output_current 高位 mA
    10, // output_current 低位 mA

    11, // battery_voltage 高位 mV
    12, // battery_voltage 低位 mV

    13, // battery_current 高位 mA
    14, // battery_current 低位 mA

    15, //  batteryPercentage
    16, //  batteryCapacity 高位
    17, //  batteryCapacity 低位

    18, //  Power_Source, 0 usb 供电，1 电池供电
    19, //  is_usb_plugged_in , usb是否插入
    20, //  is_charging,电池是否在充电， 1 正在充电

    21, //  fan_speed风扇速度 0 ~ 100

    22, //  ShutdownRequest 关机请求 0，不请求； 1，低电量关机请求； 2，按键关机请求
};

// ---------------setting----------------------
#define FanControlAddress 0x00
u8 edata fanSpeed = 0;
u8 edata lastFanSpeed = 0;

#define i2cRTCSetAddress 0x01
#define i2cRTCReadAddress 0x56
extern RTC_ConfigTypeDef rtcConfig;

// I2C
// =================================================================
/**
* i2c: (非默认io，需要配置功能脚切换)
        SDA -> P2.4
        SCL -> P2.5

* 2c 的中断号为 24

* SDA 会掉电唤醒
*/
#define I2C_SLAVE_ADDRESS 0x5A
#define SDA P24
#define SCL P25

u8 isda;       // 设备地址标志
u8 isma;       // 存储地址标志
u8 islock;     // 数据同步锁
u8 edata addr; // i2c buffer address

// ------------ I2C_Init ------------------------
void I2C_Init()
{
    P2M0 &= ~0x30;
    P2M1 &= ~0x30; // P2.4, P2.5 准双向口

    // --- 选择i2c引脚 ---
    // I2C_S1 = 0;
    // I2C_S0 = 0; // SCL/P1.5, SDA/P1.4
    I2C_S1 = 0;
    I2C_S0 = 1; // SCL_2/P2.5, SDA_2/P2.4
    // I2C_S1 = 1;
    // I2C_S0 = 0; // SCL_3/P7.7, SDA_3/P7.6
    // I2C_S1 = 1;
    // I2C_S0 = 1; // SCL_4/P3.2, SDA_4/P3.3

    I2CCFG = 0x81;                              // 使能I2C从机模式
    I2CSLADR = (I2C_SLAVE_ADDRESS << 1) & 0xFE; // 设置从机设备地址, MA=0
    I2CSLST = 0x00;                             // 复位从机状态寄存器
    I2CSLCR = 0x78;                             // 使能从机模式中断

    // --- 设置中断优先级 ---
    PI2CH = (I2C_INTERRUPT_PRIORITY >> 1) & 0x01;
    PI2C = I2C_INTERRUPT_PRIORITY & 0x01;

    EA = 1; // 使能全局中断

    isda = 1; // 用户变量初始化
    isma = 1;
    addr = 0;
    islock = 0;
    I2CTXD = dataBuffer[0];
}

// ------------ I2C_Isr (中断号 24)------------------------
/**
 * 注意中断服务尽量耗时短，主频选择高频率（40MHz）, 加快处理速度，否则数据会发生错乱
 */
void I2C_Isr() interrupt 24
{
    // --------- START事件 ---------
    if (I2CSLST & 0x40)
    {
        I2CSLST &= ~0x40;
        isda = 1; // 若为重复起始信号时必须作此设置
        islock = 1;
    }
    // --------- RECV事件 ---------
    else if (I2CSLST & 0x20)
    {
        I2CSLST &= ~0x20;
        if (isda)
        { // 处理RECV事件（RECV DEVICE ADDR）
            isda = 0;
        }
        else if (isma)
        { // 处理RECV事件（RECV MEMORY ADDR）
            isma = 0;
            addr = I2CRXD;

            if (addr == i2cRTCReadAddress)
            {
                I2CTXD = YEAR;
            }
            else
            {
                I2CTXD = dataBuffer[addr];
            }
        }
        else
        { // 处理RECV事件（RECV DATA）
            if (I2CRXD != 181)
            {
                switch (addr++)
                {
                case 0:
                    fanSpeed = I2CRXD;
                    dataBuffer[21] = fanSpeed;
                    break;
                case 1:
                    rtcConfig.Year = I2CRXD;
                    break;
                case 2:
                    rtcConfig.Month = I2CRXD;
                    break;
                case 3:
                    rtcConfig.Day = I2CRXD;
                    break;
                case 4:
                    rtcConfig.Hour = I2CRXD;
                    break;
                case 5:
                    rtcConfig.Min = I2CRXD;
                    break;
                case 6:
                    rtcConfig.Sec = I2CRXD;
                    break;
                case 7:
                    rtcConfig.Ssec = I2CRXD;
                case 8:
                    ConfigRTC(&rtcConfig);
                    break;
                default:
                    break;
                }
            }
        }
    }
    // --------- SEND事件 ---------
    else if (I2CSLST & 0x10)
    {
        I2CSLST &= ~0x10;
        if (I2CSLST & 0x02) // 接收到NAK则停止读取数据
        {
            I2CTXD = 0xff;
        }
        else // 接收到ACK则继续读取数据
        {

            if (addr >= i2cRTCReadAddress)
            {
                addr++;
                switch (addr - i2cRTCReadAddress)
                {
                case 1:
                    I2CTXD = MONTH;
                    break;
                case 2:
                    I2CTXD = DAY;
                    break;
                case 3:
                    I2CTXD = HOUR;
                    break;
                case 4:
                    I2CTXD = MIN;
                    break;
                case 5:
                    I2CTXD = SEC;
                    break;
                case 6:
                    I2CTXD = SSEC;
                    break;
                default:
                    I2CTXD = 0xff;
                    break;
                }
            }
            else
            {
                I2CTXD = dataBuffer[++addr];
            }
        }
    }
    // ---------STOP事件 ---------
    else if (I2CSLST & 0x08)
    {
        I2CSLST &= ~0x08;
        isda = 1;
        isma = 1;
        islock = 0;
    }
}

// 定时器0 (5ms) :
// =================================================================
/**
 * 按键计时
 * rgb计时
 * PowerSource检测，及充电电流开关复位
 *
 * 定时器重装载值请使用STC-ISP工具计算
 */

#define Time0Preiod 5 // 定时器0 周期 5ms

u16 edata scanTimeCount = 0;
u16 edata longPressCount = 0;
u16 edata doulePressCount = 0;
u16 edata rgbTimeCount = 0;

extern int edata dac_vol;

/**
 定时器0初始化 (模式0， 16位自动重装载， 12T)
 * 注意 使能定时器中断
*/
void Timer0_Init(void) // 5毫秒@40MHz
{
    AUXR &= 0x7F; // 定时器时钟12T模式
    TMOD &= 0xF0; // 设置定时器模式
    TL0 = 0xE5;   // 设置定时初始值
    TH0 = 0xBE;   // 设置定时初始值
    TF0 = 0;      // 清除TF0标志
    TR0 = 1;      // 定时器0开始计时

    ET0 = 1; // 使能定时器0中断
}

/** 定时器0 中断服务 */
void TM0_Isr() interrupt 1
{
    // //P23 = !P23;
    // 每次 计数 加 5ms
    scanTimeCount += Time0Preiod;
    longPressCount += Time0Preiod;
    doulePressCount += Time0Preiod;
    rgbTimeCount += Time0Preiod;

    // if (POWER_SOURCE == 0)
    // {
    //     setDac(1500);
    //     dac_vol = 1500;
    // }

    if (PowerSourceVoltageRead() < 3000)
    {
        setDac(1500);
        dac_vol = 1500;
    }
}

// 按键状态机
// =================================================================
#define BTN P33

#define KeyDetectInterval 10   // ms 按键检测间隔
#define LongPressTime2S 2000   // ms 按键长按3S时长
#define LongPressTime3S 3000   // ms 按键长按3S时长
#define LongPressTime5S 5000   // ms 按键长按3S时长
#define DoulePressInterval 200 // ms 按键双击间隔

u8 edata hasPressed = 0;

u8 edata ShutdownRequest = 0;

extern u8 edata SafeShutdownColor[];
extern u8 edata ForceShutdownColor[];

// -----------------按键状态机定义-------------------------------
enum // 定义状态机使用的枚举类型
{
    BTN_UP = (u8)0,         // 按键松开
    BTN_DOWN_SHAKE = (u8)1, // 按键按下抖动
    BTN_DOWN = (u8)2,       // 按键按下
    BTN_UP_SHAKE = 3,       // 按键松开抖动
} KeyMachineState;

enum
{
    Released = 0,                 // 按键松开
    SingleClicked = 1,            // 单击, 按下松开100ms（双击间隔）后未按下
    DouleClicked = 2,             // 双击， 按下松开100ms（双击间隔）内按下再松开
    LongPressed2S = 3,            // 长按2s， 按下2s后未松开
    LongPressed2SAndReleased = 4, // 长按2s， 按下2s后松开
    // LongPressed3S = 3, // 长按3s， 按下3s后未松开
    LongPressed5S = 5, // 长按5s， 按下5s后未松开

} KeyState;

//---------------------按键初始化--------------------------------------
void BtnInit()
{
    P3M0 &= ~0x08;
    P3M1 |= 0x08; // 设置 P3.3 为高阻输入
}

//---------------------按键检测及处理程序--------------------------------------
void KeyHandler()
{
    if (scanTimeCount > KeyDetectInterval)
    {
        // -------------- key detect --------------
        switch (KeyMachineState)
        {
        case BTN_UP:
            if (BTN == 0)
            {
                KeyMachineState = BTN_DOWN_SHAKE;
            }
            else
            {
                if (hasPressed)
                {
                    if (doulePressCount > DoulePressInterval) // 单击：按下松开后200ms
                    {
                        KeyState = SingleClicked;
                        hasPressed = 0;
                    }
                }
                else
                {
                    KeyState = Released; // 松开
                }
            }
            break;
        case BTN_DOWN_SHAKE:
            if (BTN == 0)
            {
                KeyMachineState = BTN_DOWN;
                longPressCount = 0; // 按下计时器复位
            }
            else
            {
                KeyMachineState = BTN_UP;
            }
            break;
        case BTN_DOWN:
            if (BTN == 1)
            {
                KeyMachineState = BTN_UP_SHAKE;
            }
            else
            {
                if (longPressCount > LongPressTime5S)
                {
                    KeyState = LongPressed5S;
                }
                else if (longPressCount > LongPressTime2S)
                {
                    KeyState = LongPressed2S;
                }
            }
            break;
        case BTN_UP_SHAKE:
            if (BTN == 0)
            { // 如果为按下，视为抖动，回到上一状态
                KeyMachineState = BTN_DOWN;
            }
            else
            {
                KeyMachineState = BTN_UP;

                // 按下到松开
                if (KeyState == LongPressed2S)
                {
                    KeyState = LongPressed2SAndReleased;
                    hasPressed = 0;
                }
                else if (KeyState == LongPressed5S)
                {
                    // KeyState = LongPressed5SAndReleased;
                    hasPressed = 0;
                }
                else
                {
                    if (hasPressed)
                    {
                        KeyState = DouleClicked;
                        hasPressed = 0;
                    }
                    else
                    {
                        hasPressed = 1;
                        doulePressCount = 0;
                    }
                }
                longPressCount = 0;
            }
            break;
        default:
            KeyMachineState = BTN_UP;
            break;
        }

        // -------------- 按键处理程序 --------------
        switch (KeyState)
        {
        case SingleClicked: // 单击开启输出
            KeyState = Released;
            PowerOutOpen();
            rgbWrite(0, 255, 0);
            delayMs(100);
            break;
        case DouleClicked:
            KeyState = Released;
            rgbWrite(0, 0, 255);
            delayMs(100);
            rgbWrite(0, 0, 0);
            delayMs(10);
            PowerOutClose();
            // PowerInClose();
            PowerSaveMode();
            break;
        case LongPressed2S: // 长按的rgb显示需要放在RgbHandler中，避免灯显示冲突
            KeyState = Released;
            // rgbWrite(255, 120, 0);
            // delayMs(200);
            break;
        case LongPressed2SAndReleased:
            KeyState = Released;
            rgbWrite(SafeShutdownColor[0], SafeShutdownColor[1], SafeShutdownColor[2]);
            ShutdownRequest = 2; // 2，按键关机请求
            dataBuffer[22] = ShutdownRequest;
            delayMs(100);
            break;
        case LongPressed5S: // 长按5s, 强制关闭输出，同时断开PWR
            KeyState = Released;
            rgbWrite(ForceShutdownColor[0], ForceShutdownColor[1], ForceShutdownColor[2]);
            delayMs(100);

            rgbWrite(0, 0, 0);
            delayMs(10);
            // PowerOutClose();
            // PowerInClose();
            PowerSaveMode();
            break;
        default:
            KeyState = Released;
            // rgbWrite(5, 5, 5);
            // delayMs(200);
            break;
        }

        // 检测间隔计数清零
        scanTimeCount = 0;
    }
}

// rgb 状态灯， 与按键共用定时器0 来计数
// =================================================================
extern bool edata outputState;
extern u8 edata is_usb_plugged_in;
extern u8 edata power_source;
extern u8 edata isLowPower;
extern u8 edata isCharging;

/* 优先级：充电 > 低电量 > 电池供电 > usb供电
    充电：橙色呼吸
    低电量：红色闪烁
    电池供电：黄色
    usb供电：绿色
*/
u8 edata rgbR = 0;
u8 edata rgbG = 0;
u8 edata rgbB = 0;
float edata rgbBrightness = 0; // 0~1
u8 edata rgbBlinkFlag = 0;     // 0, 灭； 1，亮
/**  lastRgbState
 * 0, 空
 * 1 isCharging
 * 2 isLowPower
 * 3 power_source
 * 4 is_usb_plugged_in
 * 5 outputClose
 * 6 others
 */
u8 edata lastRgbState = 0;

#define rgbBreathInterVal 50   // rgb 呼吸间隔
#define rgbBlinkInterval 500   // rgb 闪烁间隔 ms
#define rgbStaticInterval 3000 // rgb 静态更新间隔
u8 edata chargingColor[] = {255, 50, 0};
u8 edata lowPowerColor[] = {255, 0, 0};
u8 edata batteryPoweredColor[] = {255, 200, 0};
u8 edata usbPoweredColor[] = {0, 255, 0};
u8 edata SafeShutdownColor[] = {255, 0, 255}; // purple
u8 edata ForceShutdownColor[] = {255, 0, 0};  // red

void RgbHandler()
{
    // isCharging = 0;
    // isLowPower = 0;
    // power_source = 0;
    // is_usb_plugged_in = 0;

    if ((KeyState == LongPressed2S) || (KeyState == LongPressed2SAndReleased))
    {
        rgbWrite(SafeShutdownColor[0], SafeShutdownColor[1], SafeShutdownColor[2]);
        return;
    }
    else if (KeyState == LongPressed5S)
    {
        rgbWrite(ForceShutdownColor[0], ForceShutdownColor[1], ForceShutdownColor[2]);
        return;
    }

    // ----------------------------------------------------------------
    if (isCharging)
    {
        if (rgbTimeCount > rgbBreathInterVal || lastRgbState != 1)
        {
            lastRgbState = 1;
            if (rgbBrightness == 1)
            {
                rgbBrightness = 0;
            }
            else
            {
                rgbBrightness += 0.05;
            }
            rgbR = (u8)(rgbBrightness * (float)chargingColor[0]);
            rgbG = (u8)(rgbBrightness * (float)chargingColor[1]);
            rgbB = (u8)(rgbBrightness * (float)chargingColor[2]);
            rgbWrite(rgbR, rgbG, rgbB);
            rgbTimeCount = 0;
        }
    }
    else if (isLowPower)
    {
        if (rgbTimeCount > rgbBlinkInterval || lastRgbState != 2)
        {
            lastRgbState = 2;
            rgbBlinkFlag = !rgbBlinkFlag;
            if (rgbBlinkFlag)
            {
                rgbR = lowPowerColor[0];
                rgbG = lowPowerColor[1];
                rgbB = lowPowerColor[2];
                rgbWrite(rgbR, rgbG, rgbB);
            }
            else
            {
                rgbClose();
            }
            rgbTimeCount = 0;
        }
    }
    else if (power_source && outputState)
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 3)
        {
            lastRgbState = 3;
            rgbWrite(batteryPoweredColor[0], batteryPoweredColor[1], batteryPoweredColor[2]);
            rgbTimeCount = 0;
        }
    }
    else if (is_usb_plugged_in && outputState)
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 4)
        {
            lastRgbState = 4;
            rgbWrite(usbPoweredColor[0], usbPoweredColor[1], usbPoweredColor[2]);
            rgbTimeCount = 0;
        }
    }
    // else if (!outputState)
    // {
    //     if (rgbTimeCount > rgbStaticInterval || lastRgbState != 5)
    //     {
    //         lastRgbState = 5;
    //         rgbClose();
    //         rgbTimeCount = 0;
    //     }
    // }
    else
    {
        // if (rgbTimeCount > rgbStaticInterval || lastRgbState != 6)
        // {
        //     lastRgbState = 6;
        //     rgbWrite(120, 120, 120);
        //     rgbTimeCount = 0;
        // }
    }
}

// 定时器1 (20ms)
// =================================================================
#define Time1Preiod 20;
u8 TimeCount20Flag = 0; //

/**
 定时器1初始化 (模式0， 24位自动重装载， 12T)
 */
void Timer1_Init(void) // 20毫秒@40MHz
{
    TM1PS = 0x01; // 设置定时器时钟预分频 ( 注意:并非所有系列都有此寄存器,详情请查看数据手册 )
    AUXR &= 0xBF; // 定时器时钟12T模式
    TMOD &= 0x0F; // 设置定时器模式
    TL1 = 0xCA;   // 设置定时初始值
    TH1 = 0x7D;   // 设置定时初始值
    TF1 = 0;      // 清除TF1标志
    TR1 = 1;      // 定时器1开始计时

    ET1 = 1; // 使能定时器1中断
}

void TM1_Isr() interrupt 3
{
    // //P23 = !P23;
    TimeCount20Flag = 1; //
    // powerProcess();
}

// 电源相关 数据读取 和 处理
// =================================================================
/*
 * 电压，电流读取
 * 电量计
 * 电池充电电流控制
 * 数据保存到date
 */
extern bool edata outputState;
extern u8 edata is_usb_plugged_in;
extern u8 edata power_source;
extern u8 edata isLowPower;
extern u8 edata isCharging;

extern u16 edata usbVoltage;
extern u16 xdata usbCurrent; // USB
extern u16 edata outputVoltage;
extern u16 edata outputCurrrent; // output
extern u16 edata batteryVoltage;
extern int16 edata batteryCurrent; // battery
extern u8 edata batteryPercentage;
extern u16 edata batteryCapctiy;
extern u16 edata vccVoltage;

void powerProcess()
{
    // ---- 读取数据 ----
    vccVoltage = VccVoltageRead();
    usbVoltage = UsbVoltageRead();
    usbCurrent = UsbCurrentRead();
    outputVoltage = OutputVoltageRead();
    outputCurrrent = OutputCurrentRead();
    batteryVoltage = BatteryVoltageRead();
    batteryCurrent = BatteryCurrentRead();
    batteryCapctiy = UpdateCapacity(batteryCurrent, 20);
    batteryPercentage = UpdateBatteryPercentage();
    // ---- 电池百分比 ----
    // if (batteryVoltage > 8400)
    // {
    //     batteryPercentage = 100;
    // }
    // else if (batteryVoltage < 6200)
    // {
    //     batteryPercentage = 0;
    // }
    // else
    // {
    //     batteryPercentage = (u8)((batteryVoltage - 6200) / 2200.0 * 100);
    // }

    // batteryCapctiy = (u16)(2000 * ((batteryVoltage - 6200) / 2200.0));

    // // ---- 判断供电和电池状态 ----
    // isCharging
    if (batteryCurrent > 100)
        isCharging = 1;
    else if (batteryCurrent < 50)
        isCharging = 0;

    // isLowPower
    if (batteryPercentage < 20)
        isLowPower = 1;
    else
        isLowPower = 0;

    // is_usb_plugged_in or power_source
    if (batteryCurrent < -100)
    {
        power_source = 1;
    }
    else
    {
        power_source = 0;
    }

    if (usbVoltage < 3000)
    {
        is_usb_plugged_in = 0;
    }
    else
    {
        is_usb_plugged_in = 1;
    }

    // // ---- 充电管理 ---
    if (PowerSourceVoltageRead() < 3100)
    {
        setDac(1500);
        dac_vol = 1500;
    }
    else
    {
        ChargeManager(usbVoltage);
    }

    // -------------- 数据保存 -----------------
    // if (!islock)
    if (1)
    {
        dataBuffer[1] = vccVoltage >> 8 & 0xFF;
        dataBuffer[2] = (u8)(vccVoltage & 0xFF);

        dataBuffer[3] = usbVoltage >> 8 & 0xFF;
        dataBuffer[4] = (u8)(usbVoltage & 0xFF);

        dataBuffer[5] = usbCurrent >> 8 & 0xFF;
        dataBuffer[6] = (u8)(usbCurrent & 0xFF);

        dataBuffer[7] = outputVoltage >> 8 & 0xFF;
        dataBuffer[8] = outputVoltage & 0xFF;

        dataBuffer[9] = outputCurrrent >> 8 & 0xFF;
        dataBuffer[10] = outputCurrrent & 0xFF;

        dataBuffer[11] = batteryVoltage >> 8 & 0xFF;
        dataBuffer[12] = batteryVoltage & 0xFF;

        dataBuffer[13] = ((u16)batteryCurrent) >> 8 & 0xFF;
        dataBuffer[14] = ((u16)batteryCurrent) & 0xFF;

        dataBuffer[15] = batteryPercentage & 0xFF;

        dataBuffer[16] = ((u16)batteryCapctiy) >> 8 & 0xFF;
        dataBuffer[17] = ((u16)batteryCapctiy) & 0xFF;

        dataBuffer[18] = power_source & 0xFF;
        dataBuffer[19] = is_usb_plugged_in & 0xFF;
        dataBuffer[20] = isCharging & 0xFF;
    }
}

// 风扇处理程序
// =================================================================
void FanHandler()
{
    if (lastFanSpeed != fanSpeed)
    {
        lastFanSpeed = fanSpeed;
        FanSetSpeed(fanSpeed);
    }
}

// 树莓派关机信号处理程序
// =================================================================
void RpiShutdownHandler()
{
    if (RPI_STATE == 1)
    {
        delayMs(5); // 5ms消抖
        if (RPI_STATE == 1)
        {
            dataBuffer[22] = 0; //  ShutdownRequest 复位
            // PowerOutClose();
            // PowerInClose();
            PowerSaveMode();
        }
    }
}

// 省电模式，（使用 按键 IO 中断进行唤醒, P33 下降沿中断， ）
// =================================================================
void PowerSaveMode()
{
    u8 _flag = 0;

    /**--------------------------进入休眠前的IO处理--------------------------------------*/
    PowerInClose();
    delayMs(10); // 消抖

    P0IE = 0x00; // 关闭IO数字信号功能，降低掉电模式下的耗电
    P1IE = 0x00;
    // P2IE = 0x00;
    // P3IE = 0x1E;
    // P4IE = 0x00;

    KeyState = Released;

    /**--------------------------开启外部中断--------------------------------------*/
    // IT0 = 0; // 使能 INT0升沿和下降沿中断
    // // IT0 = 1; //使能 INT0下降沿中断
    // EX0 = 1; // 使能 INT0中断

    // IT1 = 0; // 使能 INT1上升沿和下降沿中断
    IT1 = 1; // 使能 INT1下降沿中断
    EX1 = 1; // 使能 INT1中断

    // EX2 = 1; // 使能 INT2下降沿中断
    // EX3 = 1; // 使能 INT3下降沿中断
    // EX4 = 1; // 使能 INT4下降沿中断

    /** 测试 IO口 中断唤醒 程序复位，无法使用*/
    // P3IM0 = 0x00;
    // P3IM1 = 0x00;  // P3.3 下降沿中断模式
    // P3INTE = 0x08; // 使能 P3.3 口中断
    // P3WKUE = 0x08; // 使能 P3.3 口中断掉电唤醒

    // delayMs(10); // 消抖

    scanTimeCount = 0;
    longPressCount = 0;
    doulePressCount = 0;
    rgbTimeCount = 0;

    /**--------------------------进入休眠模式--------------------------------------*/
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    // IDL = 1; // MCU进入 IDLE模式
    PD = 1;  // MCU进入掉电模式
    _nop_(); // 掉电模式被唤醒后,MCU首先会执行此语句,然后再进入中断服务程序

    _nop_();
    _nop_();
    _nop_();

    /**--------------------------唤醒后--------------------------------------*/
    PowerInHold();
    PowerOutOpen();
    rgbWrite(0, 0, 255); // 工作指示

    P0IE = 0xff;
    P1IE = 0xff;

    // if (BTN == 0) // 按键被按下
    // {
    //     delayMs(25); // 消抖
    //     if (BTN == 0)
    //     {
    //         _flag = 1;
    //         // ----- 恢复供电 -----
    //         // PowerInHold();
    //         // PowerOutOpen();
    //         P0IE = 0xff;
    //         P1IE = 0xff;
    //         rgbWrite(0, 0, 225); // 工作指示
    //         delayMs(500);
    //         // return; // error, 测试return 会导致单片机复位
    //     }
    // }

    // if (_flag == 0)
    // {
    //     PowerSaveMode(); // 如果没有return， 重新进入休眠
    // }
}

// 掉电唤醒后
// void INT1_Isr() interrupt 2
// {
//     u8 _flag = 0;

//     if (BTN == 0) // 按键被按下
//     {
//         delayMs(5); // 消抖
//         if (BTN == 0)
//         {
//             _flag = 1;
//             // ----- 恢复供电 -----
//             PowerInHold();
//             PowerOutOpen();
//             // ET0 = 1;
//             // ET1 = 1;
//             rgbWrite(0, 0, 90); // 工作指示
//             // delayMs(500);
//             P0IE = 0xff;
//             P1IE = 0xff;
//             // return;
//         }
//     }

//     if (_flag == 0)
//     {
//         // ----- 否则，重新进入掉电模式 -----
//         _nop_();
//         _nop_();
//         _nop_();
//         _nop_();
//         // IDL = 1; // MCU进入 IDLE模式
//         PD = 1; // MCU进入掉电模式
//         _nop_();
//         _nop_();
//         _nop_();
//         _nop_();
//     }
// }

// init
// =================================================================
void init()
{

    EAXFR = 1;    // 使能扩展寄存器(XFR)访问
    CKCON = 0x00; // 设置外部数据总线速度为最快
    WTST = 0x00;  // 设置程序代码等待参数，
                  // 赋值为 0 可将 CPU执行程序的速度设置为最快

    PowerIoInit();
    PowerInHold(); // 将PWR 引脚高电平，维持电池给单片机供电
    // PowerManagerAtStart();
    PowerOutOpen();

    boardID = getBoardID();
    dataBuffer[0] = boardID;

    rgbInit(); // 初始化rgb灯
    // rgbWrite(220, 200, 0); // 工作指示
    // delayMs(50);

    rgbWrite(255, 255, 255); // 工作指示
    delayMs(100);

    P2M0 &= ~0x08;
    P2M1 &= ~0x08; // P23 准双向口

    RTC_Init();
    BtnInit();
    AdcInit();
    FanInit();
    I2C_Init();
    DacInit();
    Timer0_Init();
    Timer1_Init();
    EA = 1;

    // 在adc开启后，读取电压，初始化当前电池容量
    CapacityInit();
}

// main
// =================================================================
void main()
{
    init();

    while (1)
    {
        if (TimeCount20Flag == 1)
        {

            // P23 = !P23;
            TimeCount20Flag = 0;
            KeyHandler();
            RgbHandler();
            FanHandler();
            // RpiShutdownHandler();
            powerProcess();
        }
        // delayMs(15);
    }
}
