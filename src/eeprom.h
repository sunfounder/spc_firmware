#ifndef __EEPROM_H
#define __EEPROM_H

#include "stc32g.h"

/*------------- EEPROM -------------------------------------------

型号       大小 扇区   IAP方式读/写/擦除
                      起始地址 结束地址
STC32G8K64 IAP   IAP     0:0000h   x

EEPROM 大小取决于烧录时设置的大小，这里设置为 4k，（8个扇区），则：
起始地址:0:0000h 结束地址:0:0FFFh
----------------------------------------------------------------*/
#define IAP_TPS_VAL 12 // 等待参数, 对应时钟频率12MHz

void IapIdle();
char IapRead(unsigned long addr);
void IapProgram(unsigned long addr, char dat);
void IapErase(unsigned long addr);

#endif
