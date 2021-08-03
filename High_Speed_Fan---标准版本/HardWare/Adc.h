#ifndef __ADC_H
#define __ADC_H
#include "control.h"
#define   ADC1_DR_Address    ((uint32_t)0x4001244C)       //ADC数据寄存器地址
#define   BufferLenght       36    
void ADC_INIT(void);
extern u16 hPhaseAOffset;
extern u16 hPhaseBOffset;
extern u16 hPhaseCOffset;
#endif
