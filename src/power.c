#include "power.h"

bool edata outputState = 0;
u8 edata is_usb_plugged_in = 0;
u8 edata Power_Source = 0;
u8 edata isLowPower = 0;
u8 edata isCharging = 0;

u16 edata usbVoltage = 0;
u16 xdata usbCurrent = 0; // USB
u16 edata outputVoltage = 0;
u16 edata outputCurrrent = 0; // output
u16 edata batteryVoltage = 0;
int16 edata batteryCurrent = 0; // battery
u16 edata powerSourceVoltage = 0;

u16 edata vccVoltage = 0;

u8 edata batteryPercentage = 0;
float edata batteryCapctiy = 0;

u16 refVoltage = 0;

int BGV;
int edata gain = 2;

// =========================== 初始化 ==================================
void PowerIoInit()
{
    // output 推挽输出
    P3M0 |= 0x04;
    P3M1 &= ~0x04; // PWR_CTL_CTL -> P3.2  推挽输出
    P3M0 |= 0x10;
    P3M1 &= ~0x10; // DC_EN  -> P3.4 推挽输出
    P3M0 |= 0x20;
    P3M1 &= ~0x20; // USB_EN_N -> P3.5 推挽输出
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
    P1M1 |= 0x01; // BATTERY_CURRENT -> ADC0 ->P1.4 高阻输入

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
    PWR_CTL = 1; // 维持高电平， 维持给单片机供电
}

void PowerInClose()
{
    PWR_CTL = 0; // 单片机断电
}

// ======================== 5V 输出相关 ==================================

/** 打开5v输出,记忆状态 */
void PowerOutOpen()
{
    DC_EN = 1;
    USB_EN_N = 1;
    outputState = 1;
    IapErase(POWER_MEMORY_EEPROM_ADDR);         // 擦除EEPROM扇区
    IapProgram(POWER_MEMORY_EEPROM_ADDR, 0x02); // 记忆输出打开
}

/** 打开5v输出,记忆状态 */
void PowerOutClose()
{
    DC_EN = 0;
    USB_EN_N = 0;
    outputState = 0;
    IapErase(POWER_MEMORY_EEPROM_ADDR);         // 擦除EEPROM扇区
    IapProgram(POWER_MEMORY_EEPROM_ADDR, 0x01); // 记忆输出关闭
}

/** 上电时恢复电源记忆 */
void PowerManagerAtStart()
{
    /*当 ALWAYS_ON 为有效时（低电平）时，power 输出(),
     *否则：
     *    读取eeprom值
     *    如果值为 0x01，power不输出
     *    如果值为 0x02，power不输出
     *    如果值错误，默认power不输出
     */
    char stat = 0;

    if (ALWAYS_ON == 0)
    {
        delayUs(500); // 消抖
        if (ALWAYS_ON == 0)
        {
            PowerOutOpen(); // power输出
            return;
        }
    }

    stat = IapRead(POWER_MEMORY_EEPROM_ADDR);
    // EEPROM 的写操作只能将字节中的 1 写为 0，当需要将字节中的 0 写为 1，则必须执行扇区 擦除操作。
    switch (stat)
    {
    case 0x01: // 关闭输出
        PowerOutClose();
        break;
    case 0x02: // 开启输出
        PowerOutOpen();
        break;
    default: // 默认power不输出
        PowerOutClose();
        break;
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

/** 初始化adc */
void AdcInit()
{
    EAXFR = 1; // 使能访问 XFR

    // ADCCFG &= ~0x0f; // SPEED(0)
    // ADCTIM = 0x2e;   // CSSETUP(0), CSHOLD(1), SMPDUTY(14)

    // --- 50k @ 12MHz ---
    ADCCFG &= ~0x0f;
    ADCCFG |= 0x02; // SPEED(2)
    ADCTIM = 0x38;  // CSSETUP(0), CSHOLD(1), SMPDUTY(24)

    // ADCTIM = 0x3f; // 设置 ADC内部时序
    // ADCCFG = 0x0f; // 设置 ADC时钟为系统时钟/2/16/16

    RESFMT = 1;    // RESFMT 结果右对齐
    ADC_POWER = 1; // ADC_POWER, 使能ADC模块

    BGV = (VREFH_ADDR << 8) + VREFL_ADDR; // 从 CHIPID中读取内部参考电压值;
    // BGV = REF_BGV;

    vccVoltage = VccVoltageRead();
}

/** adc读值 */
u16 AdcRead(u8 channel)
{
    u16 adc_val = 0;

    ADC_CONTR &= 0xF0;    // 复位通道
    ADC_CONTR |= channel; // 选择adc通道

    // adc 读取
    ADC_START = 1; // 启动AD转换
    _nop_();
    _nop_();
    while (!ADC_FLAG)
        ;                              // 等待ADC完成
    ADC_FLAG = 0;                      // 清完成标志
    adc_val = ADC_RES << 8 | ADC_RESL; // 读取ADC结果
    // 计算电压毫伏
    return (u16)(adc_val); // 12位adc 分辨率为 4096
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
    vccVoltage = (u16)((4096L * BGV / refVoltage));
    return vccVoltage;
}

/** 读取USB输入电压(mV)*/
u16 UsbVoltageRead()
{
    return (u16)(AdcReadVoltage(USB_VOLTAGE_CHANNEL) * USB_VOLTAGE_GAIN); // 返回读值
}

/** 读取Battery输出电压(mV) */
u16 BatteryVoltageRead()
{
    return (u16)(AdcReadVoltage(BATTERY_VOLTAGE_CHANNEL) * BATTERY_VOLTAGE_GAIN); // 返回读值
}

/** 读取output输出电压(mV) */
u16 OutputVoltageRead()
{
    return (u16)(AdcReadVoltage(OUTPUT_VOLTAGE_CHANNEL) * OUTPUT_VOLTAGE_GAIN); // 返回读值
}

u16 UsbCurrentRead()
{
    // 差分放大100倍，除以0.005欧姆电阻
    // return (u16)((AdcReadVoltage(USB_CURRENT_CHANNEL)) / 0.005 / 100); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
    return (u16)((AdcReadVoltage(USB_CURRENT_CHANNEL)) * gain); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
}

/** 读取Battery输出电流(mA)
 * 正值 充电
 * 负值 输出
 */
int16 BatteryCurrentRead()
{
    // 差分放大100倍，除以0.005欧姆电阻
    return -(int16)((AdcReadVoltage(BATTERY_CURRENT_CHANNEL) - 1250) / 0.005 / 100); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
}

/** 读取Output输出电流(mA)*/
u16 OutputCurrentRead()
{
    // 差分放大100倍，除以0.005欧姆电阻
    return (u16)((AdcReadVoltage(OUTPUT_CURRENT_CHANNEL)) / 0.005 / 100); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
}

u16 PowerSourceVoltageRead()
{
    return (u16)(AdcReadVoltage(POWER_SOURCE_CHANNEL) * POWER_SOURCE_VOLTAGE_GAIN); // 返回读值
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
#define P7Voltage 6800   // 电池 7% 电量测量点电压
#define P3Voltage 6500   // 电池 3% 电量测量点电压
#define MinVoltage 6200  // 电池 0% 电量测量点电压
#define MaxVoltage 8400  // mV 100% 时电压
#define MaxCapacity 2000 // mAh 默认电池最大容量

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

/** 上电时初始化电池剩余容量 */
void CapacityInit()
{
    batteryVoltage = BatteryVoltageRead();

    if (batteryVoltage > MaxVoltage)
    {
        batteryCapctiy = MaxCapacity;
    }
    else if (batteryVoltage < MinVoltage)
    {
        batteryCapctiy = 0;
    }
    else if (batteryVoltage > P7Voltage)
    {
        batteryCapctiy = (batteryVoltage - P7Voltage) * MaxCapacity * (1 - 0.07) / (MaxVoltage - P7Voltage);
    }
    else if (batteryVoltage < P7Voltage)
    {
        batteryCapctiy = (batteryVoltage - MinVoltage) * MaxCapacity * (0.07) / (P7Voltage - MinVoltage);
    }
}

/** 更新电量容量
 * current 电流(mA)
 * interval 计算间隔（ms）
 */
u16 UpdateCapacity(int16 current, u16 interval)
{
    batteryCapctiy += (float)current * interval / 3600 / 1000;
    if (batteryCapctiy >= MaxCapacity)
    {
        batteryCapctiy = MaxCapacity;
    }
    else if (batteryCapctiy < 0)
    {
        batteryCapctiy = 0;
    }

    return (u16)batteryCapctiy;
}

u8 UpdateBatteryPercentage()
{
    return (u8)batteryCapctiy / MaxCapacity * 100;
}
// /* ----- 低电量时 -----

// */
// void PowerManagerInLowBattery()
// {
// }

// /* ----- 电源数据读取 -----

// */
// void PowerMonitor()
// {
// }
