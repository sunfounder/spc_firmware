/*============================================================================

芯片：STC32G8K64
主频选择： 40MHZ

引脚连接:
    output:
        PWR_CTL -> P5.0
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

PowerSaveFlag = 0;

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
    P4M0 &= ~0x04;
    P4M1 |= 0x04; // P42 高阻输入
    P4M0 &= ~0x08;
    P4M1 |= 0x08; // P43 高阻输入

    delayUs(100); // !!! 注意需要加延时>50us后再读值

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

// 用于测试的引脚
// =================================================================
#define testPin P52
void testPinInit()
{
    P5M0 &= ~0x04;
    P5M1 &= ~0x04; // P52 准双向口
}

// STATUS_LED
// =================================================================
#define STATUS_LED P51

void statusLEDInit()
{
    P5M0 |= 0x01;
    P5M1 &= ~0x01; // P51, 推挽输出
}

void statusLEDOn()
{
    STATUS_LED = 1;
}

void statusLEDOff()
{
    STATUS_LED = 0;
}

// 中断优先级 （0~3级，数值越大，等级越高）
// =================================================================
#define I2C_INTERRUPT_PRIORITY 3   // PI2CH,PI2C (IP2H ^ 6, IP2 ^ 6)
#define TIME0_INTERRUPT_PRIORITY 2 // PT0H,PT0 (IPH ^ 1, IP ^ 1)
#define TIME1_INTERRUPT_PRIORITY 0 // PT1H,PT1 (IPH ^ 3, IP ^ 3)

// 函数声明
// =================================================================
void PowerSaveMode();
void RTCSettingHandler();
void Shutdown();

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

    23, // YEAR
    24, // MONT
    25, // DAY
    26, // HOUR
    27, // MIN
    28, // SEC
    29, // SSEC
};

// ---------------setting----------------------
#define maxSettingLen 12

u8 edata settingBuffer[maxSettingLen] = {
    0, // fan speed
    1, // year
    2, // month
    3, // day
    4, // hour
    5, // minute
    6, // second
    7, // 1/128 second
    8, // set rtc if value equals 1
    9,
    10,
    11,
};

#define FanControlAddress 0x00
u8 edata fanSpeed = 0;
u8 edata lastFanSpeed = 0;

#define i2cRTCSetAddress 0x01
#define i2cRTCReadStartAddress 0x56
#define i2cRTCReadAddress 0x57

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

u8 isda;                // 设备地址标志
u8 isma;                // 存储地址标志
u8 dataLock;            // 数据同步锁
u8 edata addr;          // i2c buffer address
int16 edata addrOffset; // data index offset
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
    dataLock = 0;
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
        // dataLock = 1;
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

            // if (addr == i2cRTCReadStartAddress)
            // {
            //     I2CTXD = YEAR;
            // }
            // else
            // {
            //     I2CTXD = dataBuffer[addr];
            // }
            I2CTXD = dataBuffer[addr];
        }
        else
        {
            // 处理RECV事件（RECV DATA）
            if (I2CRXD != 181 && addr <= maxSettingLen)
            {
                settingBuffer[addr++] = I2CRXD;
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
#if 0
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
#endif
            // addr++;
            // addrOffset = addr - i2cRTCReadAddress;
            // if (addrOffset < 0)
            // {
            //     I2CTXD = dataBuffer[addr];
            // }
            // // // else if (addrOffset < 6)
            // // // {
            // // //     I2CTXD = (*(unsigned char volatile far *)(0x7efe71 + addrOffset));
            // // // }
            // else
            // {
            //     I2CTXD = 0xff;
            // }
            I2CTXD = dataBuffer[++addr];
        }
    }
    // ---------STOP事件 ---------
    else if (I2CSLST & 0x08)
    {
        I2CSLST &= ~0x08;
        isda = 1;
        isma = 1;
        dataLock = 0;
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
 定时器0初始化  (模式0， 16位自动重装载， 12T)
 * 注意 使能定时器中断
 * 5毫秒@35MHz
*/
void Timer0_Init(void) // 5毫秒@35MHz
{
    AUXR &= 0x7F; // 定时器时钟12T模式
    TMOD &= 0xF0; // 设置定时器模式
    TL0 = 0x09;   // 设置定时初始值
    TH0 = 0xC7;   // 设置定时初始值
    TF0 = 0;      // 清除TF0标志
    TR0 = 1;      // 定时器0开始计时

    ET0 = 1; // 使能定时器0中断
}

/** 定时器0 中断服务 */
void TM0_Isr() interrupt 1
{
    // //testPin = !testPin;
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
    if (boardID == 0)
    {
        if (PowerSourceVoltageRead() < 3000)
        {
            setDac(1500);
            dac_vol = 1500;
        }
    }

    // time
    dataBuffer[23] = YEAR;
    dataBuffer[24] = MONTH;
    dataBuffer[25] = DAY;
    dataBuffer[26] = HOUR;
    dataBuffer[27] = MIN;
    dataBuffer[28] = SEC;
    dataBuffer[29] = SSEC;
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

extern u8 edata lastRgbState;
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
    LongPressed5S = 5,            // 长按5s， 按下5s后未松开
    LongPressed5SAndReleased = 6, // 长按5s， 按下5s后松开

} KeyState;

//---------------------按键初始化--------------------------------------
void BtnInit()
{
    // P3M0 &= ~0x08;
    // P3M1 |= 0x08; // 设置 P3.3 为高阻输入
    P3M0 &= ~0x08;
    P3M1 &= ~0x08; // P3.3 准双向口
}

//---------------------按键检测及处理程序--------------------------------------
void KeyStateHandler()
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
                    KeyState = LongPressed5SAndReleased;
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
        // 检测间隔计数清零
        scanTimeCount = 0;
    }
}

void KeyEventHandler()
{
    // -------------- 按键处理程序 --------------
    switch (KeyState)
    {
    case SingleClicked: // 单击开启输出
        KeyState = Released;
        PowerOutOpen();
        rgbWrite(0, 255, 0);
        lastRgbState = 0; // 复位rgb状态
        statusLEDOn();
        delayMs(100);
        break;
    case DouleClicked:
        KeyState = Released;
        rgbWrite(0, 0, 255);
        delayMs(100);
        rgbWrite(0, 0, 0);
        delayMs(10);
        lastRgbState = 0; // 复位rgb状态
        Shutdown();
        break;
    case LongPressed2S: // 长按的rgb显示需要放在RgbHandler中，避免灯显示冲突
        // KeyState = Released;
        // rgbWrite(255, 120, 0);
        // delayMs(200);
        break;
    case LongPressed2SAndReleased:
        KeyState = Released;
        ShutdownRequest = 2; // 2，按键关机请求
        dataBuffer[22] = ShutdownRequest;
        rgbWrite(0, 0, 0);
        delayMs(10);
        lastRgbState = 0; // 复位rgb状态
        break;
    case LongPressed5S:
        // rgbWrite(ForceShutdownColor[0], ForceShutdownColor[1], ForceShutdownColor[2]);
        // delayMs(100);
        break;
    case LongPressed5SAndReleased:
        KeyState = Released;
        rgbWrite(0, 0, 0);
        delayMs(10);
        lastRgbState = 0; // 复位rgb状态
        //
        Shutdown();
        break;
    default:
        // KeyState = Released;
        // rgbWrite(5, 5, 5);
        // delayMs(200);
        break;
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

    if ((KeyState == LongPressed2S))
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
    else if (outputState)
    {
        if (power_source) // 电池供电，黄色
        {
            if (rgbTimeCount > rgbStaticInterval || lastRgbState != 3)
            {
                lastRgbState = 3;
                rgbWrite(batteryPoweredColor[0], batteryPoweredColor[1], batteryPoweredColor[2]);
                rgbTimeCount = 0;
            }
        }
        else if (is_usb_plugged_in) // usb供电，绿色
        {
            if (rgbTimeCount > rgbStaticInterval || lastRgbState != 4)
            {
                lastRgbState = 4;
                rgbWrite(usbPoweredColor[0], usbPoweredColor[1], usbPoweredColor[2]);
                rgbTimeCount = 0;
            }
        }
        else // 未接usb，但开启输出，黄色
        {
            if (rgbTimeCount > rgbStaticInterval || lastRgbState != 3)
            {
                lastRgbState = 3;
                rgbWrite(batteryPoweredColor[0], batteryPoweredColor[1], batteryPoweredColor[2]);
                rgbTimeCount = 0;
            }
        }
    }
    else if (!outputState)
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 5)
        {
            lastRgbState = 5;
            rgbClose();
            rgbTimeCount = 0;
        }
    }
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

// 省电模式，
// 外部中断0（INT0 P32 上升沿中断唤醒）
// 外部中断1（INT1 P33 下降沿中断唤醒）
// =================================================================
u8 edata hasSleep = false; // 是否进入过休眠

void PowerSaveMode()
{
#if 0
    PowerSaveFlag = 1;
#else
    hasSleep = true;
    testPin = 0;

    /**--------------------------进入休眠前的IO处理, 数据处理--------------------------------------*/
    PowerOutClose();

    rgbWrite(255, 0, 0);
    delayMs(50);
    rgbClose();
    delayMs(1);

    statusLEDOff();

    PWMB_BKR = 0x00; // 关闭PWMB主输出

    // ENLVR = 1; // 禁用系统低压复位
    // LVDF = 0;
    // ELVD = 0; //
    // delayMs(2);

    P2M0 &= ~0x07;
    P2M1 |= 0x07;

    LED_R = 1;
    LED_G = 1;
    LED_B = 1;

    /** 关闭IO数字信号输入功能， 降低掉电模式下的耗电
     * 若 I/O 被当作比较器输入口、ADC 输入口或者触摸按键输入口等模拟口时，
     * 进入时钟停振模式前，必须设置为 0，否则会有额外的耗电。
     *
     * 若 I/O 被当作数字口时，必须设置为 1，否 MCU 无法读取外部端口的电平
     */
    P0IE = 0x00;
    P1IE = 0x00;
    P2IE = 0x00; // 关闭IO数字信号输入功能， 顺便可以关闭i2c引脚（P24, P25）的唤醒功能
    P3IE = 0x0C; // 仅保留 P32（USB, INT0）, P33（按键, INT1） 的引脚输入功能，用于唤醒
    P4IE = 0x00;

    ADC_POWER = 0; // 关闭 ADC_POWER, 降低休眠功耗

    delayMs(10);

    /**--------------------------设置外部中断INT1, 进入休眠--------------------------------------*/
    IT1 = 1; // 使能 INT1下降沿中断 // P33，按键按下，下降沿触发
    EX1 = 1; // 使能 INT1中断

    IT0 = 0; // //使能 INT0 上升沿和下降沿中断 // P32, usb上电，上升沿触发
    EX0 = 1; // 使能 INT1中断

    PowerInClose();
    // delayMs(10); // 电流表超量程后，死机
    // delayMs(5); // 电流表超量程后，死机
    delayMs(1); // 电流表跳到约100uA，再保持在约20uA
    // delayUs(200); // // 电流表约0不跳动，电池灯灭，但电池还在供电
    // delayUs(5); // 电流表约0不跳动，电池灯灭，但电池还在供电

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

    /**--------------------------唤醒后处理--------------------------------------*/
    // 使能IO数字输入功能
    P0IE = 0xff;
    P1IE = 0xff;
    P2IE = 0xff;
    P3IE = 0xff;
    P4IE = 0xff;

    P2M0 |= 0x07;
    P2M1 &= ~0x07;

    P5M0 |= 0x01; // 不加有时候会开不了电
    P5M1 &= ~0x01;
    delayMs(1);
    PowerInHold();

    PowerOutOpen();

    PWMB_BKR = 0x80; // 使能PWMB主输出
    delayMs(10);
    rgbWrite(0, 200, 0);

    ADC_POWER = 1; // 重新打开 ADC_POWER, 等待1ms后，ADC 采样稳定
    delayMs(5);    // 等待adc稳定

    delayMs(200); // 等待树莓派上电

#endif
}

// 掉电唤醒后
// 外部中断 INT1 中断服务函数
void INT1_Isr() interrupt 2
{
    // EX1 = 0; // 关闭INT1中断 // ！！！增加此句会导致出错，无法唤醒
}
// 外部中断 INT0 中断服务函数
void INT0_Isr() interrupt 0
{
    // EX0 = 0; // 关闭INT0中断
}

// 定时器1 (20ms)
// =================================================================
#define Time1Preiod 20
u8 edata TimeCount20Flag = 0; //

/**
 定时器1初始化 (模式0， 16位自动重装载， 12T)
 */
void Timer1_Init(void) // 20毫秒@35MHz
{
    AUXR &= 0xBF; // 定时器时钟12T模式
    TMOD &= 0x0F; // 设置定时器模式
    TL1 = 0x23;   // 设置定时初始值
    TH1 = 0x1C;   // 设置定时初始值
    TF1 = 0;      // 清除TF1标志
    TR1 = 1;      // 定时器1开始计时

    ET1 = 1; // 使能定时器1中断
}

void TM1_Isr() interrupt 3
{
    // //testPin = !testPin;
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
 * 电源状态
 * 低电量发送关机请求
 */
#define LOW_BATTERY_WARNING 20     // 20% 触发低电量警告（红灯闪烁）
#define LOW_BATTERY_TO_POWER_OFF 5 // 5% 触发关机请求的电量阈值
#define LOW_BATTERY_MAX_COUNT 250  // 5000 / 20, 持续5s中电池低于 5%时（避免adc测量误差问题）, 发送关机请求
u16 lowPowerCount = 0;

extern bool edata outputState;
extern u8 edata is_usb_plugged_in;
extern u8 edata power_source;
extern u8 edata isLowPower;
extern u8 edata isCharging;

extern u16 edata usbVoltage;
extern u16 edata usbCurrent; // USB
extern u16 edata outputVoltage;
extern u16 edata outputCurrrent; // output
extern u16 edata batteryVoltage;
extern int16 edata batteryCurrent; // battery
extern u8 edata batteryPercentage;
extern float edata batteryCapctiy;
extern u16 edata vccVoltage;
extern u16 edata powerSourceVoltage;

u16 _adcVal = 0;

void powerProcess()
{
    // ---- 读取数据 ----
    _adcVal = VccVoltageRead();
    if (_adcVal != 0)
        vccVoltage = _adcVal;
    _adcVal = UsbVoltageRead();
    if (_adcVal != 0)
        usbVoltage = _adcVal;
    _adcVal = UsbCurrentRead();
    if (_adcVal != 0)
        usbCurrent = _adcVal;
    _adcVal = OutputVoltageRead();
    if (_adcVal != 0)
        outputVoltage = _adcVal;
    _adcVal = OutputCurrentRead();
    if (_adcVal != 0)
        outputCurrrent = _adcVal;
    _adcVal = BatteryVoltageRead();
    if (_adcVal != 0)
        batteryVoltage = _adcVal;

    _adcVal = BatteryCurrentRead();
    if (_adcVal != 0)
        batteryCurrent = _adcVal;

    if (batteryVoltage > 6000 && batteryVoltage < 8500)
    {
        UpdateCapacity(batteryCurrent, (u16)Time1Preiod);
        UpdateBatteryPercentage();
    }

    // ---- 判断供电和电池状态 ----
    // isCharging
    if (batteryCurrent > 100)
        isCharging = 1;
    else if (batteryCurrent < 20)
        isCharging = 0;

    // isLowPower
    if (batteryPercentage < LOW_BATTERY_WARNING)
    {
        isLowPower = 1;
    }
    else
    {
        isLowPower = 0;
    }

    // is_usb_plugged_in or power_source
    if (batteryCurrent < -100)
    {
        power_source = 1; // 电池供电
    }
    else
    {
        power_source = 0; // usb供电
    }

    if (usbVoltage < 3000)
    {
        is_usb_plugged_in = 0;
        power_source = 1; // usb未插入，为电池供电
    }
    else
    {
        is_usb_plugged_in = 1;
    }

    // 低电量关机管理
    // if (batteryPercentage < LOW_BATTERY_TO_POWER_OFF && is_usb_plugged_in == 0)
    // {
    //     lowPowerCount++;
    //     if (lowPowerCount >= LOW_BATTERY_MAX_COUNT)
    //     {
    //         ShutdownRequest = 1; // 1，低电量关机请求
    //         dataBuffer[22] = ShutdownRequest;
    //     }
    // }
    // else
    // {
    //     lowPowerCount = 0;
    // }

    // ---- 充电管理 ---
    if (boardID == 0)
    {
        _adcVal = PowerSourceVoltageRead();
        if (_adcVal != 0)
        {
            powerSourceVoltage = _adcVal;
            if (powerSourceVoltage < 3100)
            {
                setDac(1500);
                dac_vol = 1500;
            }
            else
            {
                ChargeManager(usbVoltage);
            }
        }
    }

    // -------------- 数据保存 -----------------
    // if (!dataLock)
    // {
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
    // }

    // if (batteryVoltage < 6800)
    // {
    //     while (1)
    //     {
    //         delayMs(10);
    //     }
    // }
}

// 风扇处理程序
// =================================================================
fan_start_count = 0;     // 100速度100ms来启动电机
FAN_START_COUNT_MAX = 5; // 5*20 ms 约 100ms

void FanHandler()
{
    if (fanSpeed != settingBuffer[0])
    {
        fanSpeed = settingBuffer[0];
        // FanSetSpeed(fanSpeed);
        dataBuffer[21] = fanSpeed;
        fan_start_count = 0;
    }

    if (fan_start_count == 0)
    {
        FanSetSpeed(100);
        fan_start_count++;
    }
    else if (fan_start_count < FAN_START_COUNT_MAX)
    {
        fan_start_count++;
    }
    else if (fan_start_count == FAN_START_COUNT_MAX)
    {
        FanSetSpeed(fanSpeed);
        fan_start_count++;
    }
}

// RTC 设置程序
// =================================================================
void RTCSettingHandler()
{
    if (settingBuffer[8] == 1)
    {
        settingBuffer[8] = 0;

        rtcConfig.Year = settingBuffer[1];
        rtcConfig.Month = settingBuffer[2];
        rtcConfig.Day = settingBuffer[3];
        rtcConfig.Hour = settingBuffer[4];
        rtcConfig.Min = settingBuffer[5];
        rtcConfig.Sec = settingBuffer[6];
        rtcConfig.Ssec = settingBuffer[7];

        ConfigRTC(&rtcConfig);
    }
}

// 树莓派关机信号处理程序
// =================================================================
void Shutdown()
{
    dataBuffer[22] = 0; //  ShutdownRequest 复位

    if (boardID == 0 && is_usb_plugged_in == 0)
    {
        PowerSaveMode();
    }
    else
    {
        PowerOutClose();
        statusLEDOff();
    }
}

void RpiShutdownHandler()
{
    if (RPI_STATE == 1)
    {
        delayMs(5); // 5ms消抖
        if (RPI_STATE == 1)
        {
            Shutdown();
        }
    }
}

// USB 和 电池 都未接事件处理
// =================================================================
void NoPowerHandler()
{
    if (is_usb_plugged_in == 0 && batteryVoltage < 6000)
    {
        PowerSaveMode();
    }
}

// USB 未插入事件处理
// =================================================================
#define usbUnpluggedMaxCount 50 // 20ms * 50 = 1000ms = 1s
u8 edata usbUnpluggedCount = 0;

/**输出关闭时，超过1s未插入usb, 进入休眠*/
void UsbUnpluggedHandler()
{
    if (is_usb_plugged_in == 0 && outputState == 0)
    {
        usbUnpluggedCount += 1;
        if (usbUnpluggedCount > usbUnpluggedMaxCount)
        {
            usbUnpluggedCount = 0;
            PowerSaveMode();
        }
    }
    else
    {
        usbUnpluggedCount = 0;
    }
}

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

    boardID = getBoardID();
    // dataBuffer[0] = boardID;

    rgbInit();               // 初始化rgb灯
    rgbWrite(200, 200, 200); // 工作指示, 白灯

    statusLEDInit();
    statusLEDOn();

    testPinInit();
    testPin = 1;
    delayMs(20);

    BtnInit();
    AdcInit();
    FanInit();
    dataBuffer[21] = 0;
    I2C_Init();
    Timer0_Init();
    Timer1_Init();
    EA = 1;

    if (boardID == 0)
    {
        RTC_Init();
        DacInit();

        // 在adc开启后，读取电压，初始化当前电池容量
        delayMs(5); // 等待adc稳定
        CapacityInit();
    }

    // 若有电池，要等到读取电池初始化容量，才开启输出
    // PowerOutOpen();
    PowerManagerAtStart();

    rgbClose();   //
    delayMs(200); // 等待树莓派上电稳定
}

u8 flag = 0;
void main()
{
    init();

    while (1)
    {
        if (TimeCount20Flag == 1)
        {
            // testPin = !testPin;
            TimeCount20Flag = 0;
            // rgbWrite(0, 0, 255);

            powerProcess();

            NoPowerHandler();
            UsbUnpluggedHandler();

            if (hasSleep == false) // 如果休眠过则跳过唤醒后的首次运行
            {
                KeyStateHandler();
                KeyEventHandler();
                RgbHandler(); // rgb 事件放在
                              // powerProcess(), KeyStateHandler(), KeyEventHandler() 后，
                              // 使rgb快速响应状态
                FanHandler();
                RTCSettingHandler();
                RpiShutdownHandler();
            }
            else
            {
                hasSleep = false;
            }
        }

        delayUs(50);
    }
}