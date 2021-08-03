#include "control.h"
#include "observer.h"
s32 temptheta=0;
s16 Kself_omega0;
s16 EstIalpha_test=0;				
s16 IalphaError_test=0; 
s16 Ialpha_test=0;
Atan	pvTemp;
s16 PrevTheta = 0;
s16 AccumTheta = 0; 
u32 AccumThetaCnt = 0;
unsigned long Startup_Ramp = 0;
unsigned int DELTA_STARTUP_RAMP_test=0;
unsigned long EndSpeed ;
SMC smc1 = SMC_DEFAULTS;
volatile s16 Speed_fb_dpp;
__inline void CalcEstI(SMC *s)
{
	s16 Ggain = s->Gsmopos;	//Q15格式
	s32 valtemp1,valtemp2;

	valtemp1 = Ggain * (s->Valpha - s->Ealpha - s->Zalpha) ;
	valtemp2 = s->Fsmopos * s->EstIalpha;
	EstIalpha_test=s->EstIalpha = RIGHTSHIFT15(valtemp1) + RIGHTSHIFT15(valtemp2);//s->EstIalpha = (Ggain * (s->Valpha - s->Ealpha - s->Zalpha) + s->Fsmopos * s->EstIalpha)/DIV_RATIO;

	valtemp1 = Ggain * (s->Vbeta - s->Ebeta - s->Zbeta);
	valtemp2 = s->Fsmopos * s->EstIbeta;
	s->EstIbeta = RIGHTSHIFT15(valtemp1) + RIGHTSHIFT15(valtemp2);//s->EstIbeta = (Ggain * (s->Vbeta - s->Ebeta - s->Zbeta) + s->Fsmopos * s->EstIbeta)/DIV_RATIO;
	
}
__inline void CalcIError(SMC *s)
{
	s->IalphaError = s->EstIalpha - (s->Ialpha);	//Q15
	s->IbetaError = s->EstIbeta - (s->Ibeta);		//Q15
	//     s->IalphaError = -s->EstIalpha + (s->Ialpha);	//Q15
	// 	s->IbetaError = -s->EstIbeta + (s->Ibeta);		//Q15
	Ialpha_test=s->Ialpha;
	IalphaError_test=s->IalphaError;
}
__inline void CalcZalpha(SMC *s)
{
	s32 Di, Vi, Qi;

	Di = s->Kslide * s->IalphaError;
	Vi = s->MaxSMCError;

	Qi = Di / Vi;

	s->Zalpha = Qi;    //s->Zalpha = ((s->Kslide * s->IalphaError))/s->MaxSMCError;	//Q15格式
}

__inline void CalcZbeta(SMC *s)
{
	s32 Di, Vi, Qi;

	Di = s->Kslide * s->IbetaError;
	Vi = s->MaxSMCError; 

	Qi = Di / Vi;

	s->Zbeta = Qi;    //s->Zbeta = ((s->Kslide * s->IbetaError))/s->MaxSMCError;	//Q15
}


__inline void CalcBEMF(SMC *s)
{
	s32 valtemp;
	s16 KslfT = s->Kslf;	//Q15
	s16 KslfFinalT = s->KslfFinal;//Q15

	valtemp = KslfT * (s->Zalpha - s->Ealpha); 
	s->Ealpha += RIGHTSHIFT15(valtemp);//s->Ealpha = s->Ealpha + RIGHSHIFT15(valtemp);//s->Ealpha = s->Ealpha + ((KslfT * (s->Zalpha - s->Ealpha))/DIV_RATIO);

	valtemp = KslfT * (s->Zbeta - s->Ebeta);
	s->Ebeta += RIGHTSHIFT15(valtemp);//s->Ebeta = s->Ebeta + RIGHSHIFT15(valtemp);//s->Ebeta = s->Ebeta + ((KslfT * (s->Zbeta - s->Ebeta))/DIV_RATIO);

	valtemp = KslfFinalT * (s->Ealpha - s->EalphaFinal);
	s->EalphaFinal += RIGHTSHIFT15(valtemp);//s->EalphaFinal = s->EalphaFinal + RIGHSHIFT15(valtemp);//s->EalphaFinal = s->EalphaFinal + ((KslfFinalT * (s->Ealpha -s->EalphaFinal))/DIV_RATIO);

	valtemp = KslfFinalT * (s->Ebeta - s->EbetaFinal);
	s->EbetaFinal += RIGHTSHIFT15(valtemp);//s->EbetaFinal = s->EbetaFinal + RIGHSHIFT15(valtemp);//s->EbetaFinal = s->EbetaFinal + ((KslfFinalT * (s->Ebeta - s->EbetaFinal))/DIV_RATIO);
}

__inline void CalcOmegaFltred(SMC *s)
{
	s32 valtemp;
	s16 FiltOmCoefT = s->FiltOmCoef ;//Q15

	valtemp = FiltOmCoefT * (s->Omega - s->OmegaFltred); 
	s->OmegaFltred += RIGHTSHIFT15(valtemp);//Q15// 	s->OmegaFltred = s->OmegaFltred + ((FiltOmCoefT * (s->Omega - s->OmegaFltred))/DIV_RATIO);//Q15
}
void SMC_Position_Estimation (SMC *s)
{
	s32 Di, Vi, Qi;
	
	CalcEstI(s);

	CalcIError(s);

/* ***********Sliding control calculator********************/
/*******************
滑模控制也叫变结构控制，
将被控量往稳定切换面上逼近。
********************/

	if ( (s->IalphaError <= s->MaxSMCError) && (s->IalphaError >= (-s->MaxSMCError)) )
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		CalcZalpha(s);
	}
	else if ((s->IalphaError) > 0)
	{
		s->Zalpha = s->Kslide;
	}
	else
	{
		s->Zalpha = -s->Kslide;
	}

	if ( (s->IbetaError <= s->MaxSMCError) && (s->IbetaError >= (-s->MaxSMCError)) )
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		CalcZbeta(s);
	}
	else if ((s->IbetaError) > 0)
		s->Zbeta = s->Kslide;
	else
		s->Zbeta = -s->Kslide;
/************Sliding control calculator end**********************/
	
/*************************************************
对滑模控制器的输出一阶低通滤波得到反电动势，由于一阶滤波后输出的
反电动势波动比较大，影响角度的计算，因此，用于计算角度的电压矢量
再进行一次低通滤波。原理可以参考电路的RC低通滤波电路。
s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha -s->Kslf * s->Ealpha
s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta -s->Kslf * s->Ebeta
s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha- s->KslfFinal * s->EalphaFinal
s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta- s->KslfFinal * s->EbetaFinal
*************************************************/
	CalcBEMF(s);

	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)
	pvTemp.Alpha = s->EalphaFinal;
	pvTemp.Beta = s->EbetaFinal;
	
	//用锁相环代替反正切效果较好
	temptheta = Calc_Rotor_Speed(pvTemp.Alpha,pvTemp.Beta);
	s->Theta += temptheta;
	//s->Theta = -(pvTemp.IQAngle);
	temptheta = PrevTheta - s->Theta;//temptheta = s->Theta - PrevTheta;//
	
	/*转换成到(-32768,32767)这个量化角度范围，也就是(-PI,PI)*/
	if( temptheta < -32768 )
		temptheta += 65536;
	else if( temptheta > 32767 )
		temptheta -= 65536;


	AccumTheta += temptheta;//s->Theta - PrevTheta;//
	PrevTheta = s->Theta;

	AccumThetaCnt++;

	if (AccumThetaCnt == IRP_PERCALC)
	{

		s->Omega = AccumTheta;
    //s->Omega = AccumTheta;
		AccumThetaCnt = 0;
		AccumTheta = 0;

	}
	//                    Q15(Omega) * 60
	// Speed RPMs = -----------------------------
	//               SpeedLoopTime * Motor Poles
	// For example:
	//    Omega = 0.5
	//    SpeedLoopTime = 0.001
	//    Motor Poles (pole pairs * 2) = 10
	// Then:
	//    Speed in RPMs is 3,000 RPMs

	 //s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega - FilterCoeff * s->OmegaFltred
	
/*******角度一阶滤波后来计算截止频率*********/
	CalcOmegaFltred(s);

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	// 		Wc = 2*_PI*Fc.
	// 		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	// 		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	// 		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	// 		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).

/***计算截止频率，用于反电动势的计算****/
	Di = s->OmegaFltred ;//
	Vi = IRP_PERCALC  ;//
	Qi = Di /Vi;
	
	s->Kslf = Qi; 
	s->KslfFinal = s->FiltOmCoef =  s->Kslf;
/***********END***************/

	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if( s->Kslf < Kself_omega0 )//if (s->Kslf < Q15(OMEGA0 * _PI / IRP_PERCALC)) //
	{
		s->Kslf = s->KslfFinal = s->FiltOmCoef = Kself_omega0;//s->Kslf = s->KslfFinal = Q15(OMEGA0 * _PI / IRP_PERCALC);//

	}
	//s->Theta +=  s->ThetaOffset;

	return;
}
void SMCInit(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH
//F与G增益取值范围为(0,1]????
	if (Q15((RS * LOOPTIMEINSEC)) > Q15(LS))
		s->Fsmopos = Q15(0.0);
	else
		s->Fsmopos = Q15(1) - Q15(RS * LOOPTIMEINSEC / LS);

	if (Q15(LOOPTIMEINSEC) > Q15(LS))
		s->Gsmopos = Q15(0.99999);
	else
		s->Gsmopos = Q15(LOOPTIMEINSEC / LS);

	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);

	s->FiltOmCoef = 65536 * OMEGA_CUF / IRP_PERCALC;// Cutoff frequency for omega filter
					
	Kself_omega0 = 65536 * OMEGA_CUF / IRP_PERCALC;//
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	  hSpeed_P_Gain = PLL_KP_GAIN;
  hSpeed_I_Gain = PLL_KI_GAIN;
//	hSpeed_P_Gain = 88;
//  hSpeed_I_Gain = 2200;
	wMotorMaxSpeed_dpp = MOTOR_MAX_SPEED_DPP;
	EndSpeed = ENDSPEEDOPENLOOP * POLE_PAIR_NUM* 65536* LOOPTIMEINSEC * 65536/ 60.0;//
	return;
}
void SMC_Start_Ramp(SMC *s)
{
		
			switch(Start_Up_State)
			{
				case S_INIT:
							Start_Up_State = ALIGNMENT; 
				break;

				case ALIGNMENT:

				Start_Up_State = RAMP_UP;    

				break;

				case RAMP_UP:
					if (Startup_Ramp < EndSpeed)
							{
                
							  Startup_Ramp += DELTA_STARTUP_RAMP;
				    		DELTA_STARTUP_RAMP_test=DELTA_STARTUP_RAMP;
							}
							else
							{
								if((feedback_speed>MINIMUM_SPEED_RPM)||(feedback_speed<(-MINIMUM_SPEED_RPM)))
								{
								   s->EIAngle_Error=(s->EIAngle_Current-smc1.Theta);
								}
								// Start_Up_State=LOOPRUN;
							}
							hForceElAngle+= (Startup_Ramp>>16);
							hTorque_Reference = 1000;       
							Stat_Volt_q_d.qV_Component1=hTorque_Reference;
							Stat_Volt_q_d.qV_Component2=0;
				break;
				default:
				break;
			}  
}
