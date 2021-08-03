#ifndef __TIME_PWM_H
#define __TIME_PWM_H
#define CKTIM	    ((u32)72000000uL)  //fclk
#define PWM_PRSC    ((u8)0)            //TIM1 frequency dividing coefficient
#define PWM_FREQ    ((u16) 16000)      //PWM(Hz)
#define PWM_PERIOD  ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
#define REP_RATE    (1)
#define DEADTIME_NS	((u16)1000)         
#define DEADTIME    (u16)((unsigned long long)CKTIM/2 * (unsigned long long)DEADTIME_NS/1000000000uL) 
#define TRISE_NS 2550
#define SAMPLING_TIME_NS   700
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 72uL)/1000uL) 
#define TNOISE (u16)((((u16)(TNOISE_NS)) * 72uL)/1000uL)
#define TRISE (u16)((((u16)(TRISE_NS)) * 72uL)/1000uL)
#define TDEAD (u16)((DEADTIME_NS * 72uL)/1000uL)
#define MAX_TNTR_NS TRISE_NS
#define TW_AFTER ((u16)(((DEADTIME_NS+MAX_TNTR_NS)*72ul)/1000ul))
#define TW_BEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*72ul)/1000ul))+1)
#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz
#define PWM2_MODE 0
#define PWM1_MODE 1
#define SQRT_3		1.732051
#define T		    (PWM_PERIOD * 4)
#define T_SQRT3     (u16)(T * SQRT_3)
void TIM1_PWM_INIT(void);
#endif
