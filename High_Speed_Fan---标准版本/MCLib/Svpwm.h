#ifndef __SVPWM_H
#define __SVPWM_H
#include "control.h"
void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input);
#define CALC_SVPWM SVPWM_3ShuntCalcDutyCycles
extern u8  bSector;  
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void);
#define GET_PHASE_CURRENTS SVPWM_3ShuntGetPhaseCurrentValues
#define SEQUENCE_LENGHT    0x00100000
#define SECTOR_1	(u32)1
#define SECTOR_2	(u32)2
#define SECTOR_3	(u32)3
#define SECTOR_4	(u32)4
#define SECTOR_5	(u32)5
#define SECTOR_6	(u32)6
#endif
