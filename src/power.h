#ifndef __POWER_H
#define __POWER_H

#include "stc32g.h"
#include "types.h"
#include "delay.h"
#include "intrins.h"
/*---------------------------------------------------------------
    output:
        PWR_CTL -> P3.2
        DC_EN  -> P3.4
        USB_EN_N -> P3.5

    input:
        CHG -> P0.1
        POWER_SOURCE -> ADC4 -> P1.4
        ALWAYS_ON -> P2.3
        RPI_STATE (pi gpio 26)-> P2.7

    dac:
        DAC -> PWM8_3 -> P0.3
    adc:
        USB_CURRENT -> ADC5 -> P0.5
        OUTPUT_CURRENT -> ADC1 ->P1.1
        BT_LV_3V3 -> ADC3 -> P1.3
        BATTERY_CURRENT -> ADC4 ->P1.4
        OUTPUT_3V3 -> ADC5 ->P1.5
        VBUS_3V3 -> ADC7 -> P1.7

// ------------------------
    output:
        PWR_CTL -> P3.2
        DC_EN  -> P3.4
        USB_EN_N -> P3.5

    input:
        CHG -> P0.1
        POWER_SOURCE -> P1.6
        ALWAYS_ON -> P2.3
        RPI_STATE (pi gpio 26)-> P2.7

    dac:
        DAC -> PWM8_3 -> P0.3
    adc:
        USB_CURRENT -> ADC13 -> P0.5
        OUTPUT_CURRENT -> ADC14 ->P0.6
        BT_LV_3V3 -> ADC3 -> P1.3
        BATTERY_CURRENT -> ADC0 ->P1.0
        OUTPUT_3V3 -> ADC1 ->P1.1
        VBUS_3V3 -> ADC5 -> P1.5

---------------------------------------------------------------*/
#define MAIN_Fosc 35000000L // 定义主时钟 35MHz

/** --- output  ---- */
#define PWR_CTL P50  // PWR_CTL 引脚 P3.2, 开机时维持供电，保持高电平
#define DC_EN P34    // DC_EN，P3.4 树莓派供电开关(5v输出开关)
#define USB_EN_N P35 // USB_EN_N，P3.5 USB输出开关

/** --- input  ---- */
#define CHG P01          // CHG, P0.1 是否在充电，高电平有效
#define POWER_SOURCE P14 // Power Source, P1.4 是否使用电池供电，低电平有效
#define ALWAYS_ON P23    // Always_ON，P2.3 树莓派供电开关(5v输出开关)是否常开, 低电平有效
#define RPI_STATE P27    // RPI_STATE, P2.7 树莓派开机状态，高电平为关机，低电平开机

/** --- dac  ---- */
// 注意 dac 和 rgb 都使用 PWMB, 初始化时 预分频 和 自动重装载值要一致，或者仅初始化一次
// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 35MHz / 4 / 875 = 10KHz
#define _PWMB_PSCR 4     // 预分频 2
#define _PWMB_PERIOD 875 // PWM周期 (PWMA_ARR自动重装值) 600

//
#define DAC P03 // DAC，P03, 电池充电电流控制，0~1.5v, 1.5v时充电电流最大

/** --- adc  ---- */
#define USB_VOLTAGE_CHANNEL 5  // 0x05
#define USB_CURRENT_CHANNEL 13 // 0x0d

#define OUTPUT_VOLTAGE_CHANNEL 1  // 0x01
#define OUTPUT_CURRENT_CHANNEL 14 // 0x0e

#define BATTERY_VOLTAGE_CHANNEL 3 // 0x03
#define BATTERY_CURRENT_CHANNEL 0 // 0x00

#define POWER_SOURCE_CHANNEL 4 // 0x04

#define REF_VOLTAGE_CHANNEL 15 // 0x0f

#define USB_VOLTAGE_GAIN 2.0 //((float)(100 + 100) / 100.0)
#define USB_CURRENT_GAIN 2.0 //((float)(1/0.005 / 100)

#define OUTPUT_VOLTAGE_GAIN 2.0 //((float)(100 + 100) / 100.0)
#define OUTPUT_CURRENT_GAIN 2.0 //((float)(1/0.005 / 100)

#define BATTERY_VOLTAGE_GAIN 3.0 // ((float)(200 + 100) / 100.0)
#define BATTERY_CURRENT_GAIN 2.0 //((float)(1/0.005 / 100)

#define POWER_SOURCE_VOLTAGE_GAIN 2.0 //((float)(100 + 100) / 100.0)

/** --- 充电电流调节pid参数 ---- */
#define targetVoltage 4900
#define maxValue 1500

#define kp 0.08
#define ki 0.0
#define kd 0.05

// 电池电量相关
// =============================================================
#define P7Voltage 6800   // 电池 7% 电量测量点电压
#define P3Voltage 6500   // 电池 3% 电量测量点电压
#define MinVoltage 6200  // 电池 0% 电量测量点电压
#define MaxVoltage 8240  // mV 100% 时电压
#define MaxCapacity 2000 // mAh 默认电池最大容量

/** ------函数声明----------- */
void PowerIoInit();
void PowerInHold();
void PowerInClose();

void PowerOutOpen();
void PowerOutClose();
void PowerManagerAtStart();

void AdcInit();
u16 AdcRead(u8 channel);
u16 AdcReadVoltage(u8 channel);
u16 VccVoltageRead();
u16 UsbVoltageRead();
u16 UsbCurrentRead();
u16 OutputVoltageRead();
u16 OutputCurrentRead();
u16 BatteryVoltageRead(bool filter);
int16 BatteryCurrentRead(bool filter);
u16 PowerSourceVoltageRead();

void ChargeManager(u16 current_vol);
void DacInit();
void setDac(u16 voltage);
float pidCalculate(u16 current_vol);
u16 calculateBatteryIR();
void CapacityInit();
void UpdateCapacity(int16 current, u16 interval);
void UpdateBatteryPercentage();

void PowerManagerInLowBattery();
void PowerMonitor();

#endif
