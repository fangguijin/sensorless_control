#ifndef __OBSERVER_H
#define __OBSERVER_H
#include "control.h"
#include <stdbool.h>
typedef enum 
{
 S_INIT, ALIGNMENT, RAMP_UP,LOOPRUN
} Start_upStatus_t;

extern Start_upStatus_t  Start_Up_State;

#define K1 (s32) (-6753)//(-12000)             /* State Observer Gain 1 */
// Values showed on LCD display must be here multiplied by 100 
#define K2 (s32) (6753)//(+85200)           /* State Observer Gain 2 */

#define Gain1 (s32) (-1728)
#define Gain2 (s32) (13061)
#define PLL_KP_GAIN (s16)532//1130//532//(532*MOTOR_MAX_SPEED_RPM*POLE_PAIR_NUM/SAMPLING_FREQ)  //532

#define PLL_KI_GAIN (s16)12//50//12//(1506742*POLE_PAIR_NUM/SAMPLING_FREQ\

#define F1 (s16)(16378)//(2048)
#define F2 (s16)(16378)//(8192)
#define C1 (s32)((F1*RS)/(LS*SAMPLING_FREQ))
#define C2 (s32)((F1*K1)/SAMPLING_FREQ)//((F1*K1)/SAMPLING_FREQ)
#define C3 (s32)((F1*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*SAMPLING_FREQ))
#define C4 (s32)((((K2*MAX_CURRENT)/(MAX_BEMF_VOLTAGE))*F2)/(SAMPLING_FREQ))//((((K2*MAX_CURRENT)/(MAX_BEMF_VOLTAGE))*F2)/(SAMPLING_FREQ))
#define C5 (s32)((F1*MAX_VOLTAGE)/(LS*MAX_CURRENT*SAMPLING_FREQ))
//#define C6  (s16)((F2*F3*2*PI)/65536);

#define C6_COMP_CONST1  (int32_t) 1043038
#define C6_COMP_CONST2  (int32_t) 10430
#define VARIANCE_THRESHOLD        0.10
#define PERCENTAGE_FACTOR           (u16)(VARIANCE_THRESHOLD * 128)  
#define NB_CONSECUTIVE_TESTS      (u16) 60 
void STO_InitSpeedBuffer(void);
void STO_Start_Up(void);
void STO_Calc_Rotor_Angle(Volt_Components Stat_Volt_alfa_beta,
                         Curr_Components Stat_Curr_alfa_beta,
                         u16 hBusVoltage);
void STO_Calc_Speed(void);
s16 STO_Get_Speed(void);
s16 STO_Get_Electrical_Angle(void);
void STO_Init(void);
s16 Calc_Rotor_Speed(s16 hBemf_alfa, s16 hBemf_beta);
void Store_Rotor_Speed(s16 hRotor_Speed);
void STO_InitSpeedBuffer(void);
s16 STO_Get_Speed_Hz(void);
bool IsObserverConverged(void);
void STO_StartUp_Init(void);
void STO_Gains_Init(void);
extern volatile int16_t  hForceElAngle;
extern s16 hRotor_El_Angle;
extern  s16 hAngle ;
extern volatile s16 hSpeed_Reference;
extern uint8_t Is_Speed_Reliable_flag;
extern s16 hEstimatedSpeed;
extern volatile s16 hSpeed_P_Gain, hSpeed_I_Gain;
extern  s32 wMotorMaxSpeed_dpp;
#endif
