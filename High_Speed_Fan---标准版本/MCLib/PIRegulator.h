#ifndef __PIREGULATOR_H
#define __PIREGULATOR_H
#include "control.h"
#define PID_SPEED_REFERENCE_RPM   (s16)800
#define PID_SPEED_REFERENCE  (u16)(PID_SPEED_REFERENCE_RPM/6)

//#define PID_SPEED_REFERENCE_RPM   (s16)900
#define PID_SPEED_KP_DEFAULT      (s16)800
#define PID_SPEED_KI_DEFAULT      (s16)300
#define PID_SPEED_KD_DEFAULT      (s16)0


#define SP_KPDIV ((u16)(16))
#define SP_KIDIV ((u16)(256))
#define SP_KDDIV ((u16)(16))



#define PID_TORQUE_REFERENCE   (s16)1000//6000   //(N.b: that's the reference init  
                                       //value in both torque and speed control)
#define PID_TORQUE_KP_DEFAULT  (s16)110      
#define PID_TORQUE_KI_DEFAULT  (s16)2
#define PID_TORQUE_KD_DEFAULT  (s16)0

/* default values for Flux control loop */
#define PID_FLUX_REFERENCE   (s16)0

#define PID_FLUX_KP_DEFAULT  (s16)110
#define PID_FLUX_KI_DEFAULT  (s16)2
#define PID_FLUX_KD_DEFAULT  (s16)0


#define TF_KPDIV ((u16)(1024))
#define TF_KIDIV ((u16)(16384))
#define TF_KDDIV ((u16)(8192))


#define SQUARE_WAVE_PERIOD   (u16)2000 //in msec



//Settings for min frequency
#define Freq_Min         (u16)10 // 1 Hz mechanical
#define Ki_Fmin          (u16)1000 // Frequency min coefficient settings
#define Kp_Fmin          (u16)2000
#define Kd_Fmin          (u16)0

//Settings for intermediate frequency 1
#define F_1              (u16)50 // 5 Hz mechanical
#define Ki_F_1           (u16)2000 // Intermediate frequency 1 coefficient settings
#define Kp_F_1           (u16)1000
#define Kd_F_1           (u16)0

//Settings for intermediate frequency 2
#define F_2              (u16)200 // 20 Hz mechanical
#define Ki_F_2           (u16)1000 // Intermediate frequency 2 coefficient settings
#define Kp_F_2           (u16)750
#define Kd_F_2           (u16)0

//Settings for max frequency
#define Freq_Max         (u16)500 // 50 Hz mechanical
#define Ki_Fmax          (u16)500 // Frequency max coefficient settings
#define Kp_Fmax          (u16)500
#define Kd_Fmax          (u16)0
                                                                             
/********************************************************************************/      
/* Do not modify */
/* linear coefficients */                                                                             
#define alpha_Ki_1		(s32)( ((s32)((s16)Ki_F_1-(s16)Ki_Fmin)*1024) / (s32)(F_1-Freq_Min) )
#define alpha_Kp_1		(s32)( ((s32)((s16)Kp_F_1-(s16)Kp_Fmin)*1024) / (s32)(F_1-Freq_Min) )
#define alpha_Kd_1		(s32)( ((s32)((s16)Kd_F_1-(s16)Kd_Fmin)*1024) / (s32)(F_1-Freq_Min) )

#define alpha_Ki_2		(s32)( ((s32)((s16)Ki_F_2-(s16)Ki_F_1)*1024) / (s32)(F_2-F_1) )
#define alpha_Kp_2		(s32)( ((s32)((s16)Kp_F_2-(s16)Kp_F_1)*1024) / (s32)(F_2-F_1) )
#define alpha_Kd_2		(s32)( ((s32)((s16)Kd_F_2-(s16)Kd_F_1)*1024) / (s32)(F_2-F_1) )

#define alpha_Ki_3		(s32)( ((s32)((s16)Ki_Fmax-(s16)Ki_F_2)*1024) / (s32)(Freq_Max-F_2) )
#define alpha_Kp_3		(s32)( ((s32)((s16)Kp_Fmax-(s16)Kp_F_2)*1024) / (s32)(Freq_Max-F_2) )
#define alpha_Kd_3		(s32)( ((s32)((s16)Kd_Fmax-(s16)Kd_F_2)*1024) / (s32)(Freq_Max-F_2) )
void PID_Init (PID_Struct_t *PID_Torque, PID_Struct_t *PID_Flux, 
                                                        PID_Struct_t *PID_Speed);
s16 PID_Regulator(s16 hReference, s16 hPresentFeedback, PID_Struct_t *PID_Struct);
#endif
