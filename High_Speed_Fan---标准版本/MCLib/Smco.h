#ifndef __SMCO_H
#define __SMCO_H
#include "control.h"
#define _0_05DEG 	1
#define     RIGHTSHIFT15(x)   ( (x>=0) ? (x>>15) : (-((-x)>>15)) )    //负数取绝对值之后再移位，之后取相反数
#define     RIGHTSHIFT10(x)   ( (x>=0) ? (x>>10) : (-((-x)>>10)) )
#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (s32)(32768 * (Float_Value) - 0.5) \
        : (s32)(32767 * (Float_Value) + 0.5))
#define SPEEDLOOPFREQ	3000	
#define SPEEDLOOPTIME (double)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPTIMEINSEC (double)(1.0/PWM_FREQ) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned int)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SMCGAIN		     0.325	
#define MAXLINEARSMC    0.05	
#define SPEED_CUF 8000
#define THETA_AT_ALL_SPEED 150
#define OMEGA_CUF (double)(SPEED_CUF * LOOPTIMEINSEC * \
                IRP_PERCALC * POLE_PAIR_NUM * 2.0 / 60.0)

#define THETA_ALL (float)(THETA_AT_ALL_SPEED * 32768.0 /180.0 )
#define CONSTANT_PHASE_SHIFT (THETA_ALL)
#define MINSPEEDINRPM	1000
#define ENDSPEEDOPENLOOP MINSPEEDINRPM
#define OPENLOOPTIMEINSEC 5
#define DELTA_STARTUP_RAMP	(unsigned int)(MINSPEEDINRPM*POLE_PAIR_NUM*LOOPTIMEINSEC* \
							LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))
typedef struct	
{  
	s16  Valpha;   		// Input: Stationary alfa-axis stator voltage 
	s16  Ealpha;   		// Variable: Stationary alfa-axis back EMF 
	s16  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
	s16  Zalpha;      	// Output: Stationary alfa-axis sliding control 
	s16  Gsmopos;    	// Parameter: Motor dependent control gain 
	s16  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current 
	s16  Fsmopos;    	// Parameter: Motor dependent plant matrix 
	s16  Vbeta;   		// Input: Stationary beta-axis stator voltage 
	s16  Ebeta;  		// Variable: Stationary beta-axis back EMF 
	s16  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
	s16  Zbeta;      	// Output: Stationary beta-axis sliding control 
	s16  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current 
	s16  Ialpha;  		// Input: Stationary alfa-axis stator current 
	s16  IalphaError; 	// Variable: Stationary alfa-axis current error                 
	s16  Kslide;     	// Parameter: Sliding control gain 
	s16  MaxSMCError;  	// Parameter: Maximum current error for linear SMC 
	s16  Ibeta;  		// Input: Stationary beta-axis stator current 
	s16  IbetaError;  	// Variable: Stationary beta-axis current error                 
	s16  Kslf;       	// Parameter: Sliding control filter gain 
	s16  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
	s16  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
	s16  ThetaOffset;	// Output: Offset used to compensate rotor angle
	s16  Theta;			// Output: Compensated rotor angle 
	s16  Omega;     	// Output: Rotor speed
	s16  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
	s16 EIAngle_Error;
	s16 EIAngle_Current;
	s16 EIAngle_Estimate;
	} SMC;
typedef SMC *SMC_handle;
#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
typedef struct 	{ 
	        int32_t  Alpha; 
				  int32_t  Beta;			 			 
         }Atan ;
void SMCInit(SMC_handle);
void SMC_Position_Estimation(SMC_handle);
void CalcEstI(SMC_handle);
void CalcIError(SMC_handle);
void CalcZalpha(SMC_handle);
void CalcZbeta(SMC_handle);
void CalcBEMF(SMC_handle);
void CalcOmegaFltred(SMC_handle);
s16 SMC_Speed_Get(SMC *s);
s16 Calc_Rotor_Speed(s16 hBemf_alfa, s16 hBemf_beta);
extern SMC smc1 ;
void SMC_Start_Ramp(SMC_handle);
extern volatile s16 Speed_fb_dpp;
#endif