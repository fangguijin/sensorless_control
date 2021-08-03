#include "control.h"
uint8_t ADC_IRQ_flag=0;
uint16_t usart_flag=0;
uint8_t openflag=0;
uint8_t bMC1msCompleted = 0;
uint8_t Motor_Control_Mode=1;
s16 feedback_speed;
static uint8_t bMC16msCompleted = 0;
static uint8_t bUARTCnt = 0;
volatile s16 hTorque_Reference;
volatile s16 hFlux_Reference;
static Curr_Components Stat_Curr_q_d_ref_ref;
PID_Struct_t PID_Flux_InitStructure;
PID_Struct_t PID_Torque_InitStructure;
PID_Struct_t   PID_Speed_InitStructure;
int16_t hElAngledpp;	

void Moto_Stop(void)
{
  TIM_CtrlPWMOutputs(TIM1, DISABLE);

	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}
void Moto_Start(void)
{
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
}

void SysTick_Handler(void)
{
    static u8 Tick1msCnt=1;//1
    static u8 Tick16msCnt=16;

    if(--Tick1msCnt == 0)
    {
        Tick1msCnt = 1;//1
        bMC1msCompleted=1; 						//1ms
    }
    if(--Tick16msCnt == 0)
    {
        Tick16msCnt= 16;
        bMC16msCompleted=1; 					//16ms
    }

}
void MediumFrequencyTask(void)
{
    if(bMC1msCompleted == 1)
    {        
        bMC1msCompleted = 0;
			  #ifdef State_Observer
        STO_Calc_Speed();
			  #else
			  feedback_speed=STO_Get_Speed_Hz(); 
			  #endif
			  hEstimatedSpeed = STO_Get_Speed_Hz();

    }

}
s16 cnt_test1=0;
s16 cnt_test2=0;
s16 cnt_test=0;
void ADC1_2_IRQHandler(void)		//AD中断有三种情况：AD_EOC、AD_JEOC、AD_AWD
{	
	if((ADC1->SR &  ADC_FLAG_JEOC) == ADC_FLAG_JEOC) 
	{
		ADC_IRQ_flag=2;
		ADC1->SR = ~(u32)ADC_FLAG_JEOC;           //清除注入组转换完成中断标志  
		//GPIO_ResetBits(GPIOA, GPIO_Pin_5 );
    if(openflag==1)
		{	
			Moto_Start();
		 #ifdef State_Observer
			STO_Start_Up();      
			#else
			SMC_Start_Ramp(&smc1);
			#endif
			Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
			Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
       #ifdef State_Observer
			 //return ;
			 #else
			  smc1.Ialpha=Stat_Curr_alfa_beta.qI_Component1;
			  smc1.Ibeta=Stat_Curr_alfa_beta.qI_Component2;
			 #endif
			Stat_Curr_q_d = Park(Stat_Curr_alfa_beta,hElAngledpp); 
			GPIO_ResetBits(GPIOA, GPIO_Pin_5 );
			#ifdef State_Observer
		  STO_Calc_Rotor_Angle(Stat_Volt_alfa_beta,
                          Stat_Curr_alfa_beta,
                        14745);//17us
			#else
			SMC_Position_Estimation(&smc1);
			smc1.EIAngle_Estimate= smc1.Theta;
			
			if (smc1.EIAngle_Error > _0_05DEG ||smc1.EIAngle_Error < (0- _0_05DEG))
			{
			  if (smc1.EIAngle_Error < 0)
			  smc1.EIAngle_Error += _0_05DEG;
			  else
			  smc1.EIAngle_Error -= _0_05DEG;
			}
			//smc1.EIAngle_Current = smc1.EIAngle_Estimate+smc1.EIAngle_Error;
			smc1.EIAngle_Current=smc1.EIAngle_Estimate;
			#endif
			GPIO_SetBits(GPIOA, GPIO_Pin_5 );
			RevPark_Circle_Limitation();
			Stat_Volt_alfa_beta = MCM_Rev_Park(Stat_Volt_q_d,hElAngledpp);
			#ifdef State_Observer
			//return ;
			#else
			   smc1.Valpha=Stat_Volt_alfa_beta.qV_Component1;
			   smc1.Vbeta=Stat_Volt_alfa_beta.qV_Component2;
			#endif
			
			CALC_SVPWM(Stat_Volt_alfa_beta);
		}
		else
		{
		  Moto_Stop();
			Start_Up_State = S_INIT;
			hElAngledpp=0;
		}
		  
		if((Start_Up_State == RAMP_UP)||(Start_Up_State == ALIGNMENT)||(Start_Up_State == S_INIT))
                hElAngledpp = hForceElAngle;
		else if (Start_Up_State == LOOPRUN)
            {
                hElAngledpp = STO_Get_Electrical_Angle() ;   
							 
                if(Motor_Control_Mode==MOTOR_TORQUE_MODE)		
								{
								   hTorque_Reference = PID_TORQUE_REFERENCE;
									 hFlux_Reference = PID_FLUX_REFERENCE;
									
								}		
                else if(Motor_Control_Mode==MOTOR_SPEED_MODE)	
                {
								  //feedback_speed=GET_SPEED_0_1HZ;
									 feedback_speed=STO_Get_Speed_Hz(); 
									hTorque_Reference = PID_Regulator(PID_SPEED_REFERENCE,STO_Get_Speed_Hz(),&PID_Speed_InitStructure);  
                  
								}	
								Stat_Curr_q_d_ref_ref.qI_Component1=hTorque_Reference;
								
								 Stat_Volt_q_d.qV_Component1 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component1, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);
								Stat_Curr_q_d_ref_ref.qI_Component2=hFlux_Reference;
                Stat_Volt_q_d.qV_Component2 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component2, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);							
            
						}
		 // GPIO_SetBits(GPIOA, GPIO_Pin_5 );
		 if(bUARTCnt >40) //  9  17
        {
            bUARTCnt = 0;
//						UARTC_PutChar(hForceElAngle>>8);
//					  UARTC_PutChar(hForceElAngle);
//						UARTC_PutChar(-(smc1.EIAngle_Current)>>8);
//						UARTC_PutChar(-(smc1.EIAngle_Current));
					
//					  UARTC_PutChar(hRotor_El_Angle>>8);
//					  UARTC_PutChar(hRotor_El_Angle);
						UARTC_PutChar(Stat_Curr_a_b.qI_Component1>>8);
					  UARTC_PutChar(Stat_Curr_a_b.qI_Component1);
						UARTC_PutChar(Stat_Curr_a_b.qI_Component2>>8);
					  UARTC_PutChar(Stat_Curr_a_b.qI_Component2);
//							UARTC_PutChar(PID_SPEED_REFERENCE>>8);
//							UARTC_PutChar(PID_SPEED_REFERENCE);
//							UARTC_PutChar(STO_Get_Speed_Hz()>>8);
//					    UARTC_PutChar(STO_Get_Speed_Hz());
		    }
        else
        {
            bUARTCnt ++;        
        }

	}
}