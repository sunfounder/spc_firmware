#include "eeprom.h"
#include "intrins.h"

void IapIdle()
{
    IAP_CONTR = 0; // 关闭IAP功能
    IAP_CMD = 0;   // 清除命令寄存器
    IAP_TRIG = 0;  // 清除触发寄存器
    IAP_ADDRE = 0x00;
    IAP_ADDRH = 0x00;
    IAP_ADDRL = 0x00;
}

char IapRead(unsigned long addr)
{
    char dat;

    IAP_CONTR = 0x80;       // 使能IAP
    IAP_TPS = IAP_TPS_VAL;  // 设置等待参数
    IAP_CMD = 1;            // 设置IAP读命令
    IAP_ADDRL = addr;       // 设置IAP低地址
    IAP_ADDRH = addr >> 8;  // 设置IAP高地址
    IAP_ADDRE = addr >> 16; // 设置 IAP最高地址
    IAP_TRIG = 0x5a;        // 写触发命令(0x5a)
    IAP_TRIG = 0xa5;        // 写触发命令(0xa5)
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    dat = IAP_DATA; // 读IAP数据
    IapIdle();      // 关闭IAP功能

    return dat;
}

void IapProgram(unsigned long addr, char dat)
{
    IAP_CONTR = 0x80;       // 使能IAP
    IAP_TPS = IAP_TPS_VAL;  // 设置等待参数
    IAP_CMD = 2;            // 设置IAP写命令
    IAP_ADDRL = addr;       // 设置IAP低地址
    IAP_ADDRH = addr >> 8;  // 设置IAP高地址
    IAP_ADDRE = addr >> 16; // 设置 IAP最高地址
    IAP_DATA = dat;         // 写IAP数据
    IAP_TRIG = 0x5a;        // 写触发命令(0x5a)
    IAP_TRIG = 0xa5;        // 写触发命令(0xa5)
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    IapIdle(); // 关闭IAP功能
}

void IapErase(unsigned long addr)
{
    IAP_CONTR = 0x80;       // 使能IAP
    IAP_TPS = IAP_TPS_VAL;  // 设置等待参数
    IAP_CMD = 3;            // 设置IAP擦除命令
    IAP_ADDRL = addr;       // 设置IAP低地址
    IAP_ADDRH = addr >> 8;  // 设置IAP高地址
    IAP_ADDRE = addr >> 16; // 设置 IAP最高地址
    IAP_TRIG = 0x5a;        // 写触发命令(0x5a)
    IAP_TRIG = 0xa5;        // 写触发命令(0xa5)
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    IapIdle(); // 关闭IAP功能
}
