#ifndef __MOTO_PARAM_H
#define __MOTO_PARAM_H
#define	POLE_PAIR_NUM 	(u8) 2                //Defines the number of magnetic pole pairs. 极对数
#define RS                   0.30                  // Stator resistance , ohm  内阻
                                               //Defines the motor winding resistance (phase) in Ohms.
#define LS                   0.000375    
#define MOTOR_MAX_SPEED_RPM        3000
#define MOTOR_VOLTAGE_CONSTANT      6.23
#define FINAL_START_UP_SPEED      (u16)200//Rotor mechanical speed (rpm)
#define FREQ_START_UP_DURATION    (u16) 3000 //in msec
#define I_START_UP_DURATION       (u16) 1000 //in msec
#define FIRST_I_STARTUP           (u16) 2881
#define FINAL_I_STARTUP           (u16) 4361
#define FREQ_STARTUP_PWM_STEPS (u32) ((FREQ_START_UP_DURATION * SAMPLING_FREQ)\
                                                                          /1000) 
#define FREQ_INC (u16) ((FINAL_START_UP_SPEED*POLE_PAIR_NUM*65536/60)\
                                                        /FREQ_STARTUP_PWM_STEPS)
#define I_STARTUP_PWM_STEPS (u32) ((I_START_UP_DURATION * SAMPLING_FREQ)/1000)  
#define I_INC (u16)((FINAL_I_STARTUP -FIRST_I_STARTUP)*1024/I_STARTUP_PWM_STEPS)
#define MAX_VOLTAGE (s16)36             //((3.3/2)/BUS_ADC_CONV_RATIO)
#define MAX_BEMF_VOLTAGE  (u16)((MOTOR_MAX_SPEED_RPM*\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000*SQRT_3))
#define MOTOR_MAX_SPEED_DPP (s32)((1.2 * MOTOR_MAX_SPEED_RPM * 65536 * POLE_PAIR_NUM)\
                                                            /(SAMPLING_FREQ * 60))
#define MINIMUM_SPEED_RPM         (u16) 150
#define MINIMUM_SPEED               (u16) (MINIMUM_SPEED_RPM / 6)
#define MAX_CURRENT 9.6//2.9   	
#define NOMINAL_CURRENT              (s16)17806 
#define IQMAX NOMINAL_CURRENT
#endif