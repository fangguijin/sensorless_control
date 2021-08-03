#include "control.h"
#include "moto_param.h"
#include "observer.h"
#include <stdbool.h>
Start_upStatus_t  Start_Up_State = S_INIT;
volatile s16 hSpeed_Reference=1000;
 s16 hAngle = 0;
 uint8_t obser_covflg=0;
 u32 wTime = 0;
 s32 wStart_Up_Freq = 0;
static s16 hFreq_Inc;
static s32 hI_Inc;
volatile int16_t  hForceElAngle=0;
s16 FREQ_INC_test=0;
void STO_Start_Up(void)
{
  s16 hAux;
#ifdef NO_SPEED_SENSORS_ALIGNMENT
  static u32 wAlignmentTbase=0;
#endif  
  
  switch(Start_Up_State)
  {
  case S_INIT:
    //Init Ramp-up variables
    if (hSpeed_Reference >= 0)
    {
      hFreq_Inc = FREQ_INC;
      FREQ_INC_test=FREQ_INC;
    }
    else
    {
      hFreq_Inc = -(s16)FREQ_INC;
     
    }
    Start_Up_State = ALIGNMENT;
    break;
    
  case ALIGNMENT:
#ifdef NO_SPEED_SENSORS_ALIGNMENT
    wAlignmentTbase++;
    if(wAlignmentTbase <= SLESS_T_ALIGNMENT_PWM_STEPS)
    {                  
      hFlux_Reference = SLESS_I_ALIGNMENT * wAlignmentTbase / 
                                                    SLESS_T_ALIGNMENT_PWM_STEPS;               
      hTorque_Reference = 0;
      
      Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
      Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
      Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, SLESS_ALIGNMENT_ANGLE_S16);  
      /*loads the Torque Regulator output reference voltage Vqs*/   
      Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);  
      /*loads the Flux Regulator output reference voltage Vds*/
      Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure); 

      RevPark_Circle_Limitation();

      /*Performs the Reverse Park transformation,
      i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
      stationary reference frame*/

      Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

      /*Valpha and Vbeta finally drive the power stage*/ 
      CALC_SVPWM(Stat_Volt_alfa_beta);
    }
    else
    {
      wAlignmentTbase = 0;                
      Stat_Volt_q_d.qV_Component1 = Stat_Volt_q_d.qV_Component2 = 0;
      hTorque_Reference = PID_TORQUE_REFERENCE;
      hFlux_Reference = PID_FLUX_REFERENCE;
      Start_Up_State = RAMP_UP;
      hAngle = SLESS_ALIGNMENT_ANGLE_S16;      
    }
#else
    Start_Up_State = RAMP_UP;    
#endif    
    break;
    
  case RAMP_UP:
    wTime ++;  
    if (wTime <= I_STARTUP_PWM_STEPS)
    {     
      wStart_Up_Freq += hFreq_Inc;
 
    }
    else if (wTime <= FREQ_STARTUP_PWM_STEPS )
    {
      wStart_Up_Freq += hFreq_Inc;
    }       
    else
    {
     
    }
    if (IsObserverConverged()==true)
	//	if(Is_Speed_Reliable_flag==1)
    { 
       PID_Speed_InitStructure.wIntegral = (s32)(hTorque_Reference*256);			
      //STO_StartUp_Init();  
      Start_Up_State = LOOPRUN; 
			obser_covflg=1;
    }    
   
    //Add angle increment for ramp-up
    hAux = wStart_Up_Freq/65536;
    hAngle = (s16)(hAngle + (s32)(65536/(SAMPLING_FREQ/hAux)));
	  hForceElAngle=hAngle;
    hTorque_Reference = 1000;       
		Stat_Volt_q_d.qV_Component1=hTorque_Reference;
    Stat_Volt_q_d.qV_Component2=0;
    break;
    default:
    break;
  }    
}
#define BUFFER_SIZE (u8)64//64

/* Private variables ---------------------------------------------------------*/
s32 wIalfa_est,wIbeta_est;
static s32 wBemf_alfa_est, wBemf_beta_est;
static s32 wSpeed_PI_integral_sum;
s16 hBemf_alfa_est, hBemf_beta_est;

s16 hSpeed_Buffer[BUFFER_SIZE];
//static s16 hRotor_El_Angle;
//static s16 hRotor_Speed_dpp;
//static volatile s16 hSpeed_P_Gain, hSpeed_I_Gain;
s16 hRotor_El_Angle;
s16 hRotor_Speed_dpp;
volatile s16 hSpeed_P_Gain, hSpeed_I_Gain;
static bool Max_Speed_Out, Min_Speed_Out, Is_Speed_Reliable;
static u8 bSpeed_Buffer_Index;

extern const s16 hSin_Cos_Table[256];

static volatile s16 hC2, hC4;
s16 hF1, hF2, hF3, hC1, hC3,hC5, hC6;



 s32 wMotorMaxSpeed_dpp;
static u16 hPercentageFactor;

/* Private function prototypes -----------------------------------------------*/
s16 Calc_Rotor_Speed(s16, s16);
s16 Speed_PI(s16, s16);
void Store_Rotor_Speed(s16);

//function of MC_Clarke_Park.c
//Trig_Components Trig_Functions(s16);

/*******************************************************************************
* Function Name : STO_Calc_Rotor_Angle
* Description : This function implements the State Observer of a
*       	PMSM back-emfs. Thus, motor speed and rotor angle are calculated
*	        and stored in the variables hRotor_Speed,hRotor_El_Angle.
* Input : Stator voltage reference values Valpha, Vbeta (as provided by RevPark
*         through variable Stat_Volt_alfa_beta),
*	  Stator current values Ialpha,Ibeta (as provided by Clark trasformation
*	  through variable Stat_Curr_alfa_beta),  DC bus voltage.
* Output : None
* Return : None
*******************************************************************************/
s16 hRotor_Speed;
s16 leewBemf_alfa_est1=0;
s16 leewBemf_alfa_est2=0;
s16 leewBemf_beta_est1=0;
s16 leewBemf_beta_est2=0;
s16 leewIalfa_est1=0;
s16 leewIalfa_est2=0;
s16 leewIbeta_est1=0;
s16 leewIbeta_est2=0;
s32 bDirection_out = 1;
uint8_t fang_test1=0;
uint8_t fang_test2=0;



s32 wIalfa_est_Next, wIbeta_est_Next;
s32 wBemf_alfa_est_Next, wBemf_beta_est_Next;
s16 hValfa, hVbeta;
s16 hIalfa_err, hIbeta_err;
void STO_Calc_Rotor_Angle(Volt_Components Stat_Volt_alfa_beta,
                         Curr_Components Stat_Curr_alfa_beta,
                         u16 hBusVoltage)
{
//  s32 wIalfa_est_Next, wIbeta_est_Next;
//  s32 wBemf_alfa_est_Next, wBemf_beta_est_Next;
//  
//  s16 hValfa, hVbeta;
//  s16 hIalfa_err, hIbeta_err;

  s32 bDirection;
  s32 temp;
 
    
  hIalfa_err = (s16)((wIalfa_est/hF1)-Stat_Curr_alfa_beta.qI_Component1);
  hIbeta_err = (s16)((wIbeta_est/hF1)-Stat_Curr_alfa_beta.qI_Component2);
  
//  hValfa = (s16)((Stat_Volt_alfa_beta.qV_Component1*hBusVoltage)/32768);
//  hVbeta = (s16)((Stat_Volt_alfa_beta.qV_Component2*hBusVoltage)/32768);
  temp = (s32)(Stat_Volt_alfa_beta.qV_Component1*(hBusVoltage>>1));
  hValfa=  temp>>15;
  temp = (s32)(Stat_Volt_alfa_beta.qV_Component2*(hBusVoltage>>1));
  hVbeta=  temp>>15;
  /*alfa axes observer*/
  wIalfa_est_Next = (s32)(wIalfa_est-(s32)(hC1*(s16)(wIalfa_est/hF1))+
                          (s32)(hC2*hIalfa_err)+
                            (s32)(hC5*hValfa)-
                              (s32)(hC3*(s16)(wBemf_alfa_est/hF2)));
  
  wBemf_alfa_est_Next = (s32)(wBemf_alfa_est+(s32)(hC4*hIalfa_err)+
                              (s32)(hC6*hRotor_Speed_dpp*(wBemf_beta_est/(hF2*hF3))));                              

  
  /*beta axes observer*/
  wIbeta_est_Next = (s32)(wIbeta_est-(s32)(hC1*(s16)(wIbeta_est/hF1))+
                          (s32)(hC2*hIbeta_err)+
                            (s32)(hC5*hVbeta)-
                              (s32)(hC3*(s16)(wBemf_beta_est/hF2)));
  
  wBemf_beta_est_Next = (s32)(wBemf_beta_est+(s32)(hC4*hIbeta_err)-
                              (s32)(hC6*hRotor_Speed_dpp*(wBemf_alfa_est/(hF2*hF3))));                              
     
    //Extrapolation of present rotation direction, necessary for PLL 
/* 
    if (hRotor_Speed_dpp >= 0)
    {
        bDirection = 1;
        bDirection_out = 1;        
    }
    else
    {
        bDirection = -1;
        bDirection_out = -1;           
    }  
*/
    bDirection = 1;
  /*Calls the PLL blockset*/

  hBemf_alfa_est = wBemf_alfa_est / hF2;
  hBemf_beta_est = wBemf_beta_est / hF2;
    
  hRotor_Speed = Calc_Rotor_Speed((s16)(hBemf_alfa_est * bDirection),
                                  (s16)(-hBemf_beta_est * bDirection));
  
  Store_Rotor_Speed(hRotor_Speed);
      
  hRotor_El_Angle = (s16)(hRotor_El_Angle  + hRotor_Speed);
    
  /*storing previous values of currents and bemfs*/
  wIalfa_est = wIalfa_est_Next;
  wBemf_alfa_est = wBemf_alfa_est_Next;
  
  wIbeta_est = wIbeta_est_Next;
  wBemf_beta_est = wBemf_beta_est_Next;
		
  if (wBemf_alfa_est > (s32)(S16_MAX*hF2))
  {
    wBemf_alfa_est = S16_MAX*hF2;
    leewBemf_alfa_est1++;
  }
  else if (wBemf_alfa_est <= (s32)(S16_MIN*hF2))
  {
    wBemf_alfa_est = -S16_MAX * hF2;
    leewBemf_alfa_est2++;
  }
    
  if (wBemf_beta_est > (s32)(S16_MAX * hF2))
  {
    wBemf_beta_est = S16_MAX * hF2;
    leewBemf_beta_est1 ++;
  }
  else if (wBemf_beta_est <= (s32)(S16_MIN * hF2))
  {
    wBemf_beta_est = -S16_MAX * hF2;
    leewBemf_beta_est2++;
  }

  if (wIalfa_est > (s32)(S16_MAX * hF1))
  {
    wIalfa_est = S16_MAX * hF1;
    leewIalfa_est1 ++;
  }
  else if (wIalfa_est <= (s32)(S16_MIN * hF1))
  {
    wIalfa_est = -S16_MAX * hF1;
    leewIalfa_est2++;
  }
    
  if (wIbeta_est > S16_MAX * hF1)
  {
    wIbeta_est = S16_MAX * hF1;
    leewIbeta_est1 ++;
  }
  else if (wIbeta_est <= S16_MIN * hF1)
  {
    wIbeta_est = -S16_MAX * hF1;
    leewIbeta_est2 ++;
  }  
  fang_test1=1;	
}

/*******************************************************************************
* Function Name : STO_Calc_Speed
* Description : It averages the values of hRotor_Speed contained in the speed
*               buffer; motor speed is stored in the variable hRotor_Speed_dpp
* Input : None.
* Output : None.
* Return : None.
*******************************************************************************/
uint8_t Is_Speed_Reliable_flag=0;
void STO_Calc_Speed(void)
{ 
    s32 wAverage_Speed = 0;
    s32 wError;
    s32 wAverageQuadraticError = 0;
    u8 i;
    
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        wAverage_Speed += hSpeed_Buffer[i];
    }
  
    wAverage_Speed /= BUFFER_SIZE;

    hRotor_Speed_dpp = (s16)(wAverage_Speed);

    for (i = 0; i < BUFFER_SIZE; i++)
    {
        wError = hSpeed_Buffer[i] - wAverage_Speed;
         wError = (wError * wError);
        wAverageQuadraticError += (u32)(wError);
    }
  
      //It computes the measurement variance   
    wAverageQuadraticError /= BUFFER_SIZE;
  
    //The maximum variance acceptable is here calculated as ratio of average speed 
    wAverage_Speed = (s32)(wAverage_Speed * wAverage_Speed);
    wAverage_Speed = (wAverage_Speed / 128) * hPercentageFactor;

    if (wAverageQuadraticError > wAverage_Speed)
    {
        Is_Speed_Reliable = false;
			  Is_Speed_Reliable_flag=0;
    }
    else
    {
        Is_Speed_Reliable = true;
			  Is_Speed_Reliable_flag=1;
    }
		fang_test2=1;
}
bool STO_IsSpeed_Reliable(void)
{
  return(Is_Speed_Reliable);
}
/*******************************************************************************
* Function Name : STO_IsSpeed_Reliable
* Description : Return latest value of variable Is_Speed_Reliable
* Input : None
* Output : None
* Return : boolean value: TRUE if speed is reliable, FALSE otherwise.
*******************************************************************************/
s16 hEstimatedSpeed;
s16 hUpperThreshold;
s16 hLowerThreshold;
u8 leeupcnt=0;
u8 leelowcnt=0;
u8 leeupcnt1=0;
u8 leelowcnt1=0;
u8 leespdeffcnt=0;
u16 bConvCounter;
 int16_t hMecSpeed01Hzltmp2;
s16   STOGet_Speed=0;
bool IsObserverConverged(void)
{ 
        STOGet_Speed=  STO_Get_Speed();
        //hEstimatedSpeed = STO_Get_Speed_Hz();
    
    hEstimatedSpeed = (hEstimatedSpeed < 0 ? -hEstimatedSpeed : hEstimatedSpeed);  //wStart_Up_Freq
    hUpperThreshold = ((wStart_Up_Freq / 65536) * 160)/(POLE_PAIR_NUM * 16);//hAvrMecSpeed01Hz  ???????????170,ST???170?150;
    hUpperThreshold = (hUpperThreshold < 0 ? -hUpperThreshold : hUpperThreshold);
    hLowerThreshold = ((wStart_Up_Freq / 65536) *150) / (POLE_PAIR_NUM * 16);
    hLowerThreshold = (hLowerThreshold < 0 ? -hLowerThreshold : hLowerThreshold);

  // If the variance of the estimated speed is low enough...
    if(STO_IsSpeed_Reliable() == true)
	 //  if(Is_Speed_Reliable_flag==1)
    { 
        if(hEstimatedSpeed > MINIMUM_SPEED)
        {
      //...and the estimated value is quite close to the expected value... 
            if(hEstimatedSpeed >= hLowerThreshold)
            {
                if(hEstimatedSpeed <= hUpperThreshold)
                {
                    bConvCounter++;
                    if (bConvCounter >= NB_CONSECUTIVE_TESTS)
                    {
                    // ...the algorithm converged.
                        return(true);
                    }
                    else
                    {
                        leeupcnt++;
                        return(false);
                    }            
                }
                else
                { 
                    bConvCounter = 0;
                    leeupcnt1++;
                    return(false);
                }              
            }
            else
            { 
                bConvCounter = 0;
                leelowcnt++;
                return(false);
            } 
        }
        else
        { 
            bConvCounter = 0;
            leelowcnt1++;
            return(false);
        } 
    }
    else
    { 
        bConvCounter = 0;
        leespdeffcnt++;
        return(false);
    }    
}


/*******************************************************************************
* Function Name : STO_Get_Speed
* Description : It returns the motor electrical speed (dpp)
* Input : None.
* Output : None.
* Return : hRotor_Speed_dpp.
*******************************************************************************/
s16 STO_Get_Speed(void)
{
	#ifdef State_Observer
  return (hRotor_Speed_dpp);
	#else
	return (Speed_fb_dpp);
	#endif
}

/*******************************************************************************
* Function Name : STO_Get_Electrical_Angle
* Description : It returns the rotor position (electrical angle,s16) 
* Input : None.
* Output : None.
* Return : hRotor_El_Angle.
*******************************************************************************/
s16 STO_Get_Electrical_Angle(void)
{
	 #ifdef State_Observer
  return (hRotor_El_Angle);
	#else
	return (-smc1.EIAngle_Current);
	#endif
}

/*******************************************************************************
* Function Name : STO_Init
* Description : It initializes all the State Observer related variables to
*               suitable values.
* Input : None.
* Output : None.
* Return : None.
*******************************************************************************/

void STO_Init(void)
{  
  wIalfa_est = 0;
  wIbeta_est = 0;
  wBemf_alfa_est = 0;
  wBemf_beta_est = 0;
  Is_Speed_Reliable = false;
  
  wSpeed_PI_integral_sum = 0;
  Max_Speed_Out = false;
  Min_Speed_Out = false;
  
  hRotor_El_Angle = 0;            //could be used for start-up procedure
  hRotor_Speed_dpp = 0;
  
  STO_InitSpeedBuffer();
       
}

/*******************************************************************************
* Function Name : STO_Init
* Description : It initializes all the State Observer related variables to
*               suitable values.
* Input : None.
* Output : None.
* Return : None.
*******************************************************************************/
void STO_Gains_Init(void)
{
		hC1 = C1;
		hC3 = C3;
		hC5 = C5; 
  
   {
    s16 htempk;
    hF3 = 1;
    htempk = (s16)((100*65536)/(F2*2*PI));
    while (htempk != 0)
    {
      htempk /=2;
      hF3 *=2;
    }
    hC6 = (s16)((F2*hF3*2*
                                                                    PI)/65536);//10000
  }
	hC2 = C2;
  hC4 = C4;
  hSpeed_P_Gain = PLL_KP_GAIN;
  hSpeed_I_Gain = PLL_KI_GAIN; 
  hF1 = F1;
  hF2 = F2;
  wMotorMaxSpeed_dpp = MOTOR_MAX_SPEED_DPP;
  hPercentageFactor = PERCENTAGE_FACTOR;	
}
/*******************************************************************************
* Function Name : Calc_Rotor_Speed
* Description : This function implements a PLL; it receives the motor back-emfs
*				and calculates motor speed.
* Input : Motor back-emfs hBemf_alpha,hBemf_beta.
* Output : None.
* Return : Motor speed (dpp, digit-per-pwm)
*******************************************************************************/
s16 Calc_Rotor_Speed(s16 hBemf_alfa, s16 hBemf_beta)
{
    s32 wAlfa_Sin_tmp, wBeta_Cos_tmp;
    s16 hOutput;
    Trig_Components Local_Components;
/*
    if (Start_Up_State == LOOPRUN)
    {
        Local_Components = MCM_Trig_Functions(hRotor_El_Angle);    
    }
    else
    {
        Local_Components = MCM_Trig_Functions(hForceElAngle);    
    }
*/   #ifdef State_Observer   
    Local_Components = Trig_Functions(hRotor_El_Angle);
    #else
	  Local_Components = Trig_Functions(smc1.EIAngle_Estimate); 
	  #endif
    /* Alfa & Beta BEMF multiplied by hRotor_El_Angle Cos & Sin*/
    wAlfa_Sin_tmp = (s32)(hBemf_alfa * Local_Components.hSin);
    wBeta_Cos_tmp = (s32)(hBemf_beta * Local_Components.hCos);

    /* Speed PI regulator */
    hOutput = Speed_PI((s16)(wAlfa_Sin_tmp / 32768), (s16)(wBeta_Cos_tmp / 32768));  
    Speed_fb_dpp=hOutput;
    return (hOutput);
		
}

/*******************************************************************************
* Function Name : Speed_PI
* Description : It implements the PLL PI regulator.
* Input : (B-emf alpha)*sin(theta),(B-emf beta)*cos(theta).
* Output : None.
* Return : Motor speed (dpp, digit-per-pwm).
*******************************************************************************/
  s32 wSpeed_PI_error, wOutput;
  s32 wSpeed_PI_proportional_term, wSpeed_PI_integral_term;
s16 Speed_PI(s16 hAlfa_Sin, s16 hBeta_Cos)
{
//  s32 wSpeed_PI_error, wOutput;
//  s32 wSpeed_PI_proportional_term, wSpeed_PI_integral_term;
  int32_t wDischarge = 0;
  wSpeed_PI_error = hBeta_Cos - hAlfa_Sin;
  
  wSpeed_PI_proportional_term = hSpeed_P_Gain * wSpeed_PI_error;  // !!!
  wSpeed_PI_integral_term = hSpeed_I_Gain * wSpeed_PI_error;      // !!!
  
  if ( (wSpeed_PI_integral_sum >= 0) && (wSpeed_PI_integral_term >= 0) 
        && (Max_Speed_Out == false) )
  {
    if ( (s32)(wSpeed_PI_integral_sum + wSpeed_PI_integral_term) < 0)
    {
      wSpeed_PI_integral_sum = S32_MAX;
    }
    else    
    {
      wSpeed_PI_integral_sum += wSpeed_PI_integral_term;  
    }
  }
  else if ( (wSpeed_PI_integral_sum <= 0) && (wSpeed_PI_integral_term <= 0)
             && (Min_Speed_Out == false) )
  {
    if ( (s32)(wSpeed_PI_integral_sum + wSpeed_PI_integral_term) > 0)
    {
      wSpeed_PI_integral_sum = -S32_MAX;
    }
    else 
    {
      wSpeed_PI_integral_sum += wSpeed_PI_integral_term;  
    }
  }
  else
  {
    wSpeed_PI_integral_sum += wSpeed_PI_integral_term;  
  }
  
  wOutput = (wSpeed_PI_proportional_term / 16384 + wSpeed_PI_integral_sum / 262144);
  #ifdef State_Observer
  if (wOutput > wMotorMaxSpeed_dpp)
  {
    Max_Speed_Out = true;
    wOutput = wMotorMaxSpeed_dpp;		  			 	
  }
  else if (wOutput < (-wMotorMaxSpeed_dpp))
  {
    Min_Speed_Out = true;
    wOutput = -wMotorMaxSpeed_dpp;
  } 
  else 
  {
    Max_Speed_Out = false;
    Min_Speed_Out = false;
  }  
  #else
	if (wOutput > wMotorMaxSpeed_dpp)
  {
    //Max_Speed_Out = true;
		wOutput=wMotorMaxSpeed_dpp;
    wDischarge = wMotorMaxSpeed_dpp-wOutput;		  			 	
  }
  else if (wOutput < (-wMotorMaxSpeed_dpp))
  {
    wOutput=-wMotorMaxSpeed_dpp;
    wDischarge = -wMotorMaxSpeed_dpp-wOutput;
  } 
  else 
  {

  } 
   wSpeed_PI_integral_sum+=	wDischarge;
	#endif
  return ((s16)wOutput);
}

/*******************************************************************************
* Function Name : Store_Rotor_Speed
* Description : This function stores in a buffer, whose dimension is BUFFER_SIZE,
*               the last calculated value of the motor speed.
* Input : hRotor_Speed (dpp, digit-per-pwm)
* Output : None.
* Return : None.
*******************************************************************************/
void Store_Rotor_Speed(s16 hRotor_Speed)
{
  bSpeed_Buffer_Index++;
  if (bSpeed_Buffer_Index == BUFFER_SIZE) 
  {
    bSpeed_Buffer_Index = 0;
  }  
  hSpeed_Buffer[bSpeed_Buffer_Index] = hRotor_Speed;  
}
/*******************************************************************************
* Function Name : STO_InitSpeedBuffer
* Description : This function initializes the buffer for speed measurament 
* Input : None
* Output : None
* Return : None
*******************************************************************************/
void STO_InitSpeedBuffer(void)
{
  u8 i;
  
  /*init speed buffer*/
  for (i=0;i<BUFFER_SIZE;i++)
  {
    hSpeed_Buffer[i] = 0x00;
  }
  bSpeed_Buffer_Index = 0;
}

/*******************************************************************************
* Function Name : STO_Gains_Update
* Description : This function allows to write the private variables containing 
*               both observer and PLL gains;
* Input : Pointer to Sensorless_Gains type variable
* Output : None
* Return : None
*******************************************************************************/
void STO_Gains_Update(StateObserver_GainsUpdate* STO_GainsUpdateStruct)
{
  hC2 = STO_GainsUpdateStruct->hC2; 
  hC4 = STO_GainsUpdateStruct->hC4; 
  hSpeed_P_Gain = STO_GainsUpdateStruct->PLL_P;    
  hSpeed_I_Gain = STO_GainsUpdateStruct->PLL_I; 
}

/*******************************************************************************
* Function Name : STO_Get_wIalfa_est
* Description : This function allows to export the variables containing the 
*               observed current Ialfa in s16 format
* Input : None
* Output : None
* Return : observed Ialfa in s16 format
*******************************************************************************/
s16 STO_Get_wIalfa_est(void)
{
 return((s16)(wIalfa_est/hF1));
}
/*******************************************************************************
* Function Name : STO_Get_wIbeta_est
* Description : This function allows to export the variables containing the 
*               observed current Ibeta in s16 format
* Input : None
* Output : None
* Return : observed Ibeta in s16 format
*******************************************************************************/
s16 STO_Get_wIbeta_est(void)
{
 return((s16)(wIbeta_est/hF1));
}

/*******************************************************************************
* Function Name : STO_Get_wBemf_alfa_est
* Description : This function allows to export the variables containing the 
*               observed Bemf alfa in s16 format
* Input : None
* Output : None
* Return : observed Bemf alfa in s16 format
*******************************************************************************/
s16 STO_Get_wBemf_alfa_est(void)
{
 return((s16)(hBemf_alfa_est));
}

/*******************************************************************************
* Function Name : STO_Get_wBemf_beta_est
* Description : This function allows to export the variables containing the 
*               observed Bemf beta in s16 format
* Input : None
* Output : None
* Return : observed Bemf alfa in s16 format
*******************************************************************************/
s16 STO_Get_wBemf_beta_est(void)
{
 return((s16)(hBemf_beta_est));
}

s16 STO_Get_Speed_Hz(void)
{
	 #ifdef State_Observer
  return (s16)((STO_Get_Speed() * SAMPLING_FREQ * 10)/(65536 * POLE_PAIR_NUM));
	#else
	 return (-(s16)((STO_Get_Speed() * SAMPLING_FREQ * 10)/(65536 * POLE_PAIR_NUM)));
	#endif
}

void STO_StartUp_Init(void)
{
  //Re_initialize Start Up
  Start_Up_State = S_INIT;  
  hAngle = 0;
  wTime = 0;
  wStart_Up_Freq = 0;
  bConvCounter = 0; 
}