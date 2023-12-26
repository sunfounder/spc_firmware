#include "power.h"

// power status
bool edata outputState = 0;
u8 edata is_usb_plugged_in = 0;
u8 edata is_bat_plugged_in = 0;
u8 edata Power_Source = 0;
u8 edata isLowPower = 0;
u8 edata isCharging = 0;

// adc power Voltage Current
u16 edata usbVoltage = 0;
u16 edata usbCurrent = 0; // USB
u16 edata outputVoltage = 0;
u16 edata outputCurrrent = 0; // output
u16 edata batteryVoltage = 0;
int16 edata batteryCurrent = 0; // battery, ！！！注意这是有符号的
u16 edata powerSourceVoltage = 0;

u16 edata vccVoltage = 0; // ADC_Vref+

// battery capcity
u8 edata batteryPercentage = 0;
float edata batteryCapctiy = 0;

u16 refVoltage = 0;

int BGV;

// adc filter
// =================================================================
// Median filter
// u32 edata batteryVoltageFilterSum = 0;
// u16 edata batteryVoltageFilterIndex = 0;
// u16 edata batteryVoltageFilterBuffer[MedianFilterSize] = {0};
// u16 edata batteryVoltageFilterMin = 0;
// u16 edata batteryVoltageFilterMax = 0;

// error count filter
#define BatVoltMaxVaildError 500 // 最大有效偏差，正负500
#define BatVoltMaxErrorCount 5   // 连续5次，超过最大偏差，则认为是有效数值
u16 lastVaildBatVolt = 0;        //
u8 batVoltErrorCount = 0;        //
int32 batVoltError = 0;          // 注意有符号

#define BatCurrMaxVaildError 500 // 最大有效偏差，正负500
#define BatCurrMaxErrorCount 5   // 连续5次，超过最大偏差，则认为是有效数值
int16 lastVaildBatCurr = 0;      // 注意有符号
u8 batCurrErrorCount = 0;        //
u32 batCurrError = 0;            // 注意有符号

// =========================== IO模式初始化 ==================================
void PowerIoInit()
{
    // output 推挽输出
    P5M0 |= 0x01;
    P5M1 &= ~0x01; // PWR_CTL_CTL -> P5.0  推挽输出
    P3M0 |= 0x10;
    P3M1 &= ~0x10; // DC_EN  -> P3.4 推挽输出
    P3M0 |= 0x20;
    P3M1 &= ~0x20; // USB_EN_N -> P3.5 推挽输出
    // 默认output 关闭
    PWR_CTL = 0;
    DC_EN = 0;
    USB_EN_N = 0;
    // input 高阻输入
    P0M0 &= ~0x02;
    P0M1 |= 0x02; // CHG -> P0.1 高阻输入
    P2M0 &= ~0x08;
    P2M1 |= 0x08; // ALWAYS_ON -> P2.3 高阻输入
    P2M0 &= ~0x80;
    P2M1 |= 0x80; // RPI_STATE (pi gpio 27)-> P2.7 高阻输入
    // dac 推挽输出
    P0M0 |= 0x08;
    P0M1 &= ~0x08; //  DAC -> PWM8_3 -> P0.3 推挽输出
    // adc 高阻输入
    P0M0 &= ~0x20;
    P0M1 |= 0x20; // USB_CURRENT -> ADC13 -> P0.5 高阻输入
    P0M0 &= ~0x40;
    P0M1 |= 0x40; // OUTPUT_CURRENT -> ADC14 ->P0.6 高阻输入
    P1M0 &= ~0x01;
    P1M1 |= 0x01; // BATTERY_CURRENT -> ADC0 ->P1.0 高阻输入

    P1M0 &= ~0x02;
    P1M1 |= 0x02; // OUTPUT_3V3 -> ADC1 ->P1.1 高阻输入
    P1M0 &= ~0x08;
    P1M1 |= 0x08; // BT_LV_3V3 -> ADC3 -> P1.3 高阻输入
    P1M0 &= ~0x10;
    P1M1 |= 0x10; // POWER_SOURCE -> ADC4 -> P1.4 高阻输入
    P1M0 &= ~0x20;
    P1M1 |= 0x20; // VBUS_3V3 -> ADC5 -> P1.5 高阻输入
}

/** 维持电池给单片机供电 */
void PowerInHold()
{
    PWR_CTL = 1; // 维持高电平， 维持电池给单片机供电
}

void PowerInClose()
{
    PWR_CTL = 0; // 电池断电
}

// ======================== 5V 输出相关 ==================================

/** 打开5v输出,记忆状态 */
void PowerOutOpen()
{
    DC_EN = 1;
    USB_EN_N = 1;
    outputState = 1;
}

/** 打开5v输出,记忆状态 */
void PowerOutClose()
{
    DC_EN = 0;
    USB_EN_N = 0;
    outputState = 0;
}

/** 上电时恢复电源记忆 */
void PowerManagerAtStart()
{
    u8 isON = 0;
    /*当 ALWAYS_ON 为有效时（低电平）时：power 输出(),
     *否则：关闭输出
     */
    if (ALWAYS_ON == 0)
    {
        delayUs(50); // 消抖
        if (ALWAYS_ON == 0)
        {
            PowerOutOpen(); // power输出
            return;
            // isON = 1;
        }
    }

    // 若没有return
    if (isON == 0)
    {
        PowerOutClose();
    }
    if (UsbVoltageRead() > 3000) // 接了usb
    {
        PowerOutClose();
    }
    else // 未接了usb, 按下按键启动时，开启输出
    {
        PowerOutOpen(); // power输出
    }
}

// ========================  adc读值相关 ==================================
/* ------------------------------------------------------------
ADC_CONTR: ADC_CONTR ADC 控制寄存器
        7           6            5          4          3 2 1 0
        ADC_POWER  ADC_START   ADC_FLAG    ADC_EPWMT   ADC_CHS[3:0]
    ADC_CHS[3:0]： 通道选择

ADC_RES： ADC 转换结果高位寄存器
ADC_RESL： ADC 转换结果低位寄存器
ADCCFG：ADC 配置寄存器
        5
       RESFMT
ADCTIM: ADC 时序控制寄存器
------------------------------------------------------------ */
#define VREFH_ADDR CHIPID7
#define VREFL_ADDR CHIPID8
u16 _adcVal = 0;

// void AdcSetRate(void) // 50KSPS@35MHz
// {
//     ADCCFG &= ~0x0f;
//     ADCCFG |= 0x06; // SPEED(6)
//     ADCTIM = 0xff;  // CSSETUP(1), CSHOLD(3), SMPDUTY(31)
// }

void AdcSetRate(void) // 74.468KSPS@35MHz
{
    ADCCFG &= ~0x0f;
    ADCCFG |= 0x04; // SPEED(4)
    ADCTIM = 0x3f;  // CSSETUP(0), CSHOLD(1), SMPDUTY(31)
}

// void AdcSetRate(void) // 100KSPS@35MHz
// {
//     ADCCFG &= ~0x0f;
//     ADCCFG |= 0x03; // SPEED(3)
//     ADCTIM = 0x3c;  // CSSETUP(0), CSHOLD(1), SMPDUTY(28)
// }

// void AdcSetRate(void) // 186.17KSPS@35MHz
// {
//     ADCCFG &= ~0x0f;
//     ADCCFG |= 0x01; // SPEED(1)
//     ADCTIM = 0x3f;  // CSSETUP(0), CSHOLD(1), SMPDUTY(31)
// }

// void AdcSetRate(void) // 200KSPS@35MHz
// {
//     ADCCFG &= ~0x0f;
//     ADCCFG |= 0x01; // SPEED(1)
//     ADCTIM = 0x3c;  // CSSETUP(0), CSHOLD(1), SMPDUTY(28)
// }

/** 初始化adc */
void AdcInit()
{
    EAXFR = 1; // 使能访问 XFR

    // AdcSetRate();

    ADCCFG = 0x2f; // 设置 ADC 时钟为系统时钟/2/16
    ADCTIM = 0x3f; // 设置 ADC 内部时序，ADC采样时间建议设最大值

    RESFMT = 1;    // RESFMT 结果右对齐
    EADC = 0;      // 关闭adc中断
    ADC_POWER = 1; // ADC_POWER, 使能ADC模块

    BGV = (VREFH_ADDR << 8) + VREFL_ADDR; // 从 CHIPID中读取内部参考电压值;
    // BGV = REF_BGV;

    VccVoltageRead();
}

/** adc读值 */
u16 AdcRead(u8 channel)
{
    u8 i = 0;
    u16 adc_val = 0;
    u16 count = 2000;

    ADC_CONTR &= 0xF0;    // 复位通道
    ADC_CONTR |= channel; // 选择adc通道

#if 0 // 读3次值，丢弃前两次值，避免切换通道造成采样电容的残存电压影响
    for (i = 0; i < 3; i++)
    {
        // count = 2000;
        // adc 读取
        ADC_START = 1; // 启动AD转换
        _nop_();
        _nop_();

        // while (!ADC_FLAG && count--)
        // { // 等待ADC完成
        //     // delayUs(1);
        //     _nop_();
        //     _nop_();
        //     _nop_();
        //     _nop_();
        //     _nop_();
        // }
        while (!ADC_FLAG)
        { // 等待ADC完成
        }
        ADC_FLAG = 0; // 清完成标志
    }
#else
    // adc 读取
    ADC_START = 1; // 启动AD转换
    _nop_();
    _nop_();
    // _nop_();
    // _nop_();

    while (!ADC_FLAG && count--)
    { // 等待ADC完成
        // delayUs(1);
        _nop_();
        _nop_();
        _nop_();
        _nop_();
        _nop_();
    }

    ADC_FLAG = 0; // 清完成标志
#endif

    if (count != 0)
    {
        adc_val = (ADC_RES & 0x0f) << 8 | ADC_RESL;
        if (adc_val > 4095)
        {
            return 0; // 数据错误,返回0
        }
        return (u16)(adc_val); // 12位adc 分辨率为 4096
    }
    else
    {
        return 0; // 超时,返回0
    }
}

u16 AdcReadVoltage(u8 channel)
{
    u16 voltage = 0;

    refVoltage = AdcRead(REF_VOLTAGE_CHANNEL);
    voltage = AdcRead(channel);
    voltage = (((float)BGV / refVoltage) * voltage);
    return (u16)(voltage);
}

u16 VccVoltageRead()
{
    refVoltage = AdcRead(REF_VOLTAGE_CHANNEL);
    _adcVal = (u16)((4096L * BGV / refVoltage));
    if (_adcVal != 0)
    {
        vccVoltage = _adcVal;
    }
    return vccVoltage;
}

/** 读取USB输入电压(mV)*/
u16 UsbVoltageRead()
{
    // return (u16)(AdcReadVoltage(USB_VOLTAGE_CHANNEL));
    _adcVal = (u16)(AdcReadVoltage(USB_VOLTAGE_CHANNEL) * USB_VOLTAGE_GAIN); // 返回读值
    if (_adcVal != 0)
    {
        usbVoltage = _adcVal;
    }
    return usbVoltage;
}

/** 读取USB输入电流(mA)*/
u16 UsbCurrentRead()
{
    // 差分放大100倍，除以0.005欧姆电阻
    // return (u16)((AdcReadVoltage(USB_CURRENT_CHANNEL)) / 0.005 / 100);
    _adcVal = (u16)((AdcReadVoltage(USB_CURRENT_CHANNEL)) * USB_CURRENT_GAIN);
    if (_adcVal != 0)
    {
        usbCurrent = _adcVal;
    }
    return usbCurrent;
}

/** 读取output输出电压(mV) */
u16 OutputVoltageRead()
{
    _adcVal = (u16)(AdcReadVoltage(OUTPUT_VOLTAGE_CHANNEL) * OUTPUT_VOLTAGE_GAIN); // 返回读值
    if (_adcVal != 0)
    {
        outputVoltage = _adcVal;
    }
    return outputVoltage;
}

/** 读取Output输出电流(mA)*/
u16 OutputCurrentRead()
{
    // 差分放大100倍，除以0.005欧姆电阻
    _adcVal = (u16)((AdcReadVoltage(OUTPUT_CURRENT_CHANNEL)) * OUTPUT_CURRENT_GAIN);
    if (_adcVal != 0)
    {
        outputCurrrent = _adcVal;
    }
    return outputCurrrent;
}

/** 读取Battery输出电压(mV)
 *
 * filter 是否开启过滤
 */

u16 BatteryVoltageRead(bool filter)
{
    _adcVal = (u16)(AdcReadVoltage(BATTERY_VOLTAGE_CHANNEL) * BATTERY_VOLTAGE_GAIN);
    if (_adcVal != 0)
    {
        if (filter)
        {
            batVoltError = _adcVal - lastVaildBatVolt;
            if (batVoltError > BatVoltMaxVaildError || batVoltError < -BatVoltMaxVaildError)
            {
                batVoltErrorCount += 1;
                if (batVoltErrorCount < BatVoltMaxErrorCount)
                {
                    return batteryVoltage; // 结束
                }
            }
        }

        // valid 值
        batteryVoltage = _adcVal;
        lastVaildBatVolt = batteryVoltage;
        batVoltErrorCount = 0;
    }
    return batteryVoltage;
}

/** 读取Battery输出电流(mA)
 * 正值 充电
 * 负值 输出
 *
 * filter 是否开启过滤
 */
int16 BatteryCurrentRead(bool filter)
{
    // 差分放大100倍，除以0.005欧姆电阻
    _adcVal = -(int16)((AdcReadVoltage(BATTERY_CURRENT_CHANNEL) - 1250) * BATTERY_CURRENT_GAIN); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
    if (_adcVal != 0)
    {
        if (filter)
        {
            batCurrError = _adcVal - lastVaildBatCurr;
            if (batCurrError > BatCurrMaxVaildError || batCurrError < -BatCurrMaxVaildError)
            {
                batCurrErrorCount += 1;
                if (batCurrErrorCount < BatCurrMaxErrorCount)
                {
                    return batteryCurrent; // 结束
                }
            }
        }

        // valid 值
        batteryCurrent = _adcVal;
        lastVaildBatCurr = batteryCurrent;
        batCurrErrorCount = 0;
    }
    return batteryCurrent;
}

u16 PowerSourceVoltageRead()
{
    _adcVal = (u16)(AdcReadVoltage(POWER_SOURCE_CHANNEL) * POWER_SOURCE_VOLTAGE_GAIN); // 返回读值
    if (_adcVal != 0)
    {
        powerSourceVoltage = _adcVal;
    }
    return powerSourceVoltage;
}

// ======================== 电池充电相关 ==================================
/** dac初始化 */
void DacInit()
{
    u8 PWMB_CCER2_buff = 0;
    /* DAC -> PWM8_3 -> P0.3
     * 注意 dac 和 rgb 都使用 PWMB, 初始化时 预分频 和 自动重装载值要一致，或者仅初始化一次
     */
    // P0M0 |= 0x08;
    // P0M1 &= ~0x08; // DAC, P0.3 推挽输出
    P0M0 &= ~0x08;
    P0M1 &= ~0x08; // DAC, P0.3 准双向口

    P_SW2 |= 0x80; // 使能扩展寄存器(XFR)访问

    // PWMB_PS = 0x80; //  设置pwm8 输出引脚 P03
    PWMB_PS &= ~0xC0;
    PWMB_PS |= 0x80; //  设置pwm8 输出引脚 P03

    PWMB_CCER2_buff = PWMB_CCER2; // 保存PWM7设置
    PWMB_CCER2 = 0x00;            // 写CCMRx前必须先清零CCERx关闭通道

    PWMB_CCMR4 = 0x68; // 设置 PWM8 输出模式 1, 使能预装载功能

    // PWMB_CCER2 |= 0x10; // 使能PWM8P通道, 极性为 高电平有效
    // PWMB_CCER2 = 0x10; // 使能PWM7通道, 设置极性为 低电平有效, 使能PWM8P通道, 极性为 高电平有效
    PWMB_CCER2 = PWMB_CCER2_buff | 0x10; // 使能PWM8P通道, 极性为 高电平有效

    PWMB_PSCRH = (u8)((_PWMB_PSCR - 1) >> 8); // 设置PWMB预分频
    PWMB_PSCRL = (u8)(_PWMB_PSCR - 1);
    PWMB_ARRH = (u8)((_PWMB_PERIOD - 1) >> 8); // 设置周期时间， PWMB_ARR自动重装值
    PWMB_ARRL = (u8)(_PWMB_PERIOD - 1);

    PWMB_CCR8H = (u8)(1500 >> 8); // 初始化PWM8占空比时间 1.5v 停止充电
    PWMB_CCR8L = (u8)(1500);

    PWMB_ENO |= 0x40; // 使能 PWM8 输出

    PWMB_BKR = 0x80; // 使能PWMB主输出
    PWMB_CR1 = 0x01; // 使能PWMB计数器，开始计时, (00 边沿对齐模式)
}

/** 设置dac输出电压（mV）*/
void setDac(u16 voltage)
{
    u16 ccr = 0;
    ccr = ((u32)_PWMB_PERIOD * voltage / vccVoltage); // 满级电压3.3V

    P_SW2 |= 0x80;               // 使能扩展寄存器(XFR)访问
    PWMB_CCR8H = (u8)(ccr >> 8); // 设置PWM1占空比
    PWMB_CCR8L = (u8)(ccr);
}

/** 电池充电管理 */
int edata dac_vol = 1500;
int edata pidOutput = 0;
void ChargeManager(u16 current_vol)
{
    /* 如果,不需要对外部供电, 则:最大电流对电池充电
     * 否则:根据usb输入电压，使用pid调节dac电压，进而调节充电电流
     */
    pidOutput = (int)pidCalculate(current_vol);
    dac_vol -= pidOutput;
    if (dac_vol > 1500)
        dac_vol = 1500;
    else if (dac_vol < 0)
        dac_vol = 0;
    setDac((u16)dac_vol);

    // setDac(0);
}

/** pid计算
 * 输入：充电口(usb)电压（mV）
 * 返回：充电电流调节电压（mV）
 */
int edata lastError = 0;
long edata sumError = 0;

float pidCalculate(u16 current_vol)
{
    float error, dError, output;

    error = (int)current_vol - targetVoltage; // 偏差

    // sumError += error;                   // 积分
    // if (sumError < -330000)
    //     sumError = -330000; // 积分限幅
    // else if (sumError > 330000)
    //     sumError = 330000;
    dError = error - lastError; // 微分
    lastError = error;

    // output = kp * error + ki * sumError + kd * dError;
    output = kp * error + kd * dError;
    // if (output < 0)
    //     output = 0;
    // else if (output > maxValue)
    //     output = maxValue;

    return output;
}

// 电池电量相关
// =============================================================
u8 P7caliFlag = 0;
u16 lastVaildBatteryVoltage = 0;

// u16 BatteryIR = 50;           // mOhm 电池内阻

// /** 计算电池内阻 */
// u16 calculateBatteryIR()
// {
//     /* 通过测量开启负载后的前后压差和电流差，计算内阻
//      * IR = (U2 - U1)/(I2 - I1)
//      */
//     u16 U1, U2, I1, I2, IR;

//     // ---- 计算内阻
//     // closeRL(); // 关闭负载电阻
//     DC_EN = 0;
//     USB_EN_N = 0;
//     U1 = (u16)BatteryVoltageRead(); // mV
//     I1 = (u16)BatteryCurrentRead(); // mAh

//     // openRL(); // 打开负载电阻
//     DC_EN = 1;
//     USB_EN_N = 1;
//     U2 = (u16)BatteryVoltageRead();
//     I2 = (u16)BatteryCurrentRead();

//     IR = 1000 * (u32)(U2 - U1) / (I2 - I1); // mOhm

//     BatteryIR = IR;
//     return IR;
// }

/** 上电时初始化电池剩余容量
 *
 *  需要先初始化adc，并等待adc电源稳定>2ms,
 */
#define SamplingTimes 10
void CapacityInit()
{
    u16 _batVolt = 0;
    uint32 _batVoltSum = 0;
    u8 validCount = 0;
    u8 i = 0;

    for (i = 0; i < SamplingTimes; i++)
    {
        // 读值
        _batVolt = BatteryVoltageRead(false);
        // 检查数据是否有效 （如果>6v 且 < 8.5V）有效
        if (_batVolt > 6000 && _batVolt < 8500)
        {
            validCount++;
            _batVoltSum += _batVolt;
        }
    }

    // 如果数据有效个数>5
    if (validCount > 5)
    {
        _batVolt = _batVoltSum / validCount;
        lastVaildBatteryVoltage = _batVolt;
        is_bat_plugged_in = true;
        // 计算容量
        if (_batVolt > MaxVoltage)
        {
            batteryCapctiy = MaxCapacity;
        }
        else if (_batVolt < MinVoltage)
        {
            batteryCapctiy = 0;
        }
        else if (_batVolt > P7Voltage)
        {
            // 注意计算顺序 和 数据类型、范围
            // batteryCapctiy = MaxCapacity*0.7 + (float)(_batVolt - P7Voltage) / (MaxVoltage - P7Voltage) * MaxCapacity * (1 - 0.07);
            batteryCapctiy = 140 + (_batVolt - P7Voltage) / 1440.0 * 1860;
        }
        else if (_batVolt < P7Voltage)
        {
            // batteryCapctiy = (float)(_batVolt - MinVoltage) / (P7Voltage - MinVoltage) * MaxCapacity * (0.07);
            batteryCapctiy = (float)(_batVolt - MinVoltage) / 600 * 140;
        }
    }
    else // 数据无效，容量为0
    {
        batteryCapctiy = 0.0;
    }
}
/** 更新电量容量
 * current 电流(mA)
 * interval 计算间隔（ms）
 */

void UpdateCapacity(int16 current, u16 interval)
{
    int16 _vol_err = 0;

    // 数据不正确，或无电池，跳过电量计算
    _vol_err = (int16)(batteryVoltage - lastVaildBatteryVoltage);
    if (_vol_err > 500 || _vol_err < -500)
    {
        return;
    }
    else
    {
        lastVaildBatteryVoltage = batteryVoltage;
    }

    // calibrate the capacity at 7 % point
    if (batteryVoltage < P7Voltage && P7caliFlag == 0)
    {
        P7caliFlag = 1;
        batteryCapctiy = (float)MaxCapacity * 0.07;
    }
    else if (batteryVoltage > P7Voltage + 500)
    {
        P7caliFlag = 0;
    }
    // current integral
    batteryCapctiy += current * (float)interval / 3600.0 / 1000.0;
    if (batteryCapctiy >= MaxCapacity)
    {
        batteryCapctiy = (float)MaxCapacity;
    }
    else if (batteryCapctiy <= 0)
    {
        batteryCapctiy = 0.0;
    }
}

void UpdateBatteryPercentage()
{
    batteryPercentage = (u8)(batteryCapctiy / (float)MaxCapacity * 100);
}
