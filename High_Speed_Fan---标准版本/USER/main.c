#include "control.h"
/************************************************
 ALIENTEK 战舰STM32F103开发板实验0
 工程模板
 注意，这是手册中的新建工程章节使用的main文件 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
void SysTickConfig(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  if (SysTick_Config(SystemCoreClock /10000))   //1000 means 1ms, 2000 means 0.5ms, 10000 means 100us
  {	
    while (1);
  }
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}


uint16_t test_usart=0; ;
void MCboot(void)
{
  
	uart_init(256000);
	TIM1_PWM_INIT();
	ADC_INIT();
	GPIO_Trigger();
	#ifdef State_Observer
	STO_Gains_Init();
	STO_Init();
	#else
	SMCInit(&smc1);
	#endif
  PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure, &PID_Speed_InitStructure);  //初始化PID值
}
 
 int main(void)
 {
	
 	SysTickConfig();
	MCboot();
	UARTC_PutChar(0x99);
	UARTC_PutChar(0x55);
  while(1)
	{   
//		    test_usart++;
//		    if(test_usart==5000)
//				{
//				  test_usart=0;
//				}
		    MediumFrequencyTask();
		  
				UARTC_HandleTxd();
//			UARTC_PutChar(Stat_Curr_a_b.qI_Component1>>8);
//			UARTC_PutChar(Stat_Curr_a_b.qI_Component1);
//			UARTC_PutChar(Stat_Curr_a_b.qI_Component2>>8);
//			UARTC_PutChar(Stat_Curr_a_b.qI_Component2);
		  //  UARTC_PutChar(test_usart);
	   
	}
 }
