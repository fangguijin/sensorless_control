#include "control.h"

 void TIM1_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	  TIM_OCInitTypeDef TIM1_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;                 //计数周期
    TIM_TimeBaseStructure.TIM_Prescaler = PWM_PRSC;                //分频系数
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;        //设置外部时钟TIM1ETR的滤波时间
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;  //中央对齐模式1，从0计数到 TIM_Period 然后开始减到0，循环
    TIM_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;        //重复计数，就是重复溢出多少次才产生一个溢出中断（产生更新事件，用来触发ADC采样）
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              //配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //使能CHx的PWM输出
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//互补输出使能，使能CHxN的PWM输出
   
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //CHx有效电平的极性为高电平(高侧)
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;    //CHxN有效电平的极性为高电平(低侧)
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   //在空闲时CHx输出低(高侧), 调用TIM_CtrlPWMOutputs(TIM1, DISABLE)后就是空闲状态。
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;   //在空闲时CHxN输出高(低侧)，打开低侧管子可以用来锁电机	
	
	  TIM_OCInitStructure.TIM_Pulse = 800;                           //设置跳变值，当计数器计数到这个值时，电平发生跳变                                                               //TIM_OCIdleState 和 TIM_OCNIdleState不能同时为高
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                       //配置CH1
 
    TIM_OCInitStructure.TIM_Pulse = 800;                         
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);                       //配置CH2
 
    TIM_OCInitStructure.TIM_Pulse = 800;                         
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);                       //配置CH3
	
	TIM_OCStructInit(&TIM1_OCInitStructure);
	TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
	TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
	TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD - 1; 

	TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM1_OCInitStructure.TIM_OCNPolarity =TIM_OCNPolarity_Low;         
	TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;            
  
  TIM_OC4Init(TIM1, &TIM1_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);	  //使用TIM1的更新事件作为触发输出，这个输出可以触发ADC进行采样，电流环的采样
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);                            //清除中断标志位
	TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);                         //PWM输出使能    
	TIM_Cmd(TIM1, ENABLE);                                    //使能TIM1
}
  void TIM1_PWM_INIT(void)
  {
    TIM1_GPIO_CONFIG();
    TIM1_Mode_Config();
  }
	void TIM1_UP_IRQHandler(void)
	{
	  
	  TIM_ClearFlag(TIM1, TIM_FLAG_Update); 
	}