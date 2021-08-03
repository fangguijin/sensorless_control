#ifndef __CONTROL_H
#define __CONTROL_H
#define MOTOR_TORQUE_MODE	      0
#define MOTOR_SPEED_MODE		    1
#define	MOTOR_POSITION_MODE			2
#include "sys.h" 
#include "adc.h"
#include "time_pwm.h"
#include "usart.h"
#include "math.h"
#include "observer.h"
#include "piregulator.h"
#include "svpwm.h"
#include "type.h"
#include "delay.h"
#include "stm32f10x.h"
#include "sys.h"
#include "gpio.h"
#include "moto_param.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "smco.h"
#define GET_SPEED_0_1HZ         STO_Get_Speed_Hz()
#define PID_TORQUE_REFERENCE   (s16)1000                                    
#define PID_FLUX_REFERENCE   (s16)0
#define  State_Observer
extern  uint8_t bMC1msCompleted ;
extern uint8_t bMC16msCompleted;
void MediumFrequencyTask(void);
extern volatile s16 hTorque_Reference;
extern volatile s16 hSpeed_Reference;
extern volatile s16 hFlux_Reference;
extern PID_Struct_t PID_Flux_InitStructure;
extern PID_Struct_t PID_Torque_InitStructure;
extern PID_Struct_t   PID_Speed_InitStructure;
extern s16 feedback_speed;
#endif
