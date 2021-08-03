#include "control.h"
volatile u32  ADC_DualConvertedValueTab[BufferLenght];
volatile u16  ADC1_RegularConvertedValueTab[BufferLenght];
volatile u16  ADC2_RegularConvertedValueTab[BufferLenght];
u16 hPhaseAOffset;
u16 hPhaseBOffset;
u16 hPhaseCOffset;
static u16 bIndex;
void ADC_Mode_Config(void)
{
		ADC_InitTypeDef       ADC_InitStructure;	
		DMA_InitTypeDef       DMA_InitStructure;
		NVIC_InitTypeDef      NVIC_InitStructure;
	
	
	  
	//设置DMA1，用于自动存储ADC1和ADC2规则通道的转换值
//    DMA_DeInit(DMA1_Channel1);
//    DMA_StructInit(&DMA_InitStructure);
//    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;                  //ADC数据寄存器地址(源地址)
//    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_DualConvertedValueTab;  //转换值存储地址(目标地址)
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                           //从外设到内存
//    DMA_InitStructure.DMA_BufferSize = BufferLenght;                             //传输大小
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //外设地址不增
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                      //内存地址自增
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;      //外设数据单位(每次传输32位)
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;              //内存数据单位(每次传输32位)
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                              //循环模式
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                          //本DMA通道优先级(用了多个通道时，本参数才有效果)
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 //没有使用内存到内存的传输
//    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//	
//    DMA_ClearITPendingBit(DMA1_IT_TC1);                //清除通道1传输完成中断          
//    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);    //打开通道1传输完成中断	
//	  DMA_Cmd(DMA1_Channel1, ENABLE);                    //使能DMA1
	
	  ADC_DeInit(ADC1);
		//ADC_DeInit(ADC2);
		ADC_Cmd(ADC1, ENABLE);
		//ADC_Cmd(ADC2, ENABLE);
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;  //ADC1工作在混合同步规则及注入模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;           //轮流采集各个通道的值
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     //连续转换模式，触发后就会一直转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //转换触发信号选择，使用一个软件信号触发ADC1
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;  //数据左对齐，ADC是12位，要存到DR寄存器的高16位或低16位，就有左右对齐问题，决定了高4位无效或低4位无效
    ADC_InitStructure.ADC_NbrOfChannel = 3;	               
    ADC_Init(ADC1, &ADC_InitStructure);
		
//		ADC_StructInit(&ADC_InitStructure);  
//		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
//		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
//		ADC_InitStructure.ADC_NbrOfChannel = 1;
//		ADC_Init(ADC2, &ADC_InitStructure);
	
//    ADC_DMACmd(ADC1, ENABLE);                              //使能ADC1的DMA
//   	ADC_RegularChannelConfig(ADC1, POT_CHANNEL, 5, ADC_SampleTime_7Cycles5); 
//	  ADC_RegularChannelConfig(ADC1, BusVolt_CHANNEL, 4, ADC_SampleTime_7Cycles5); 
	
   	                           //ADC1使能
    
	
//    ADC_ResetCalibration(ADC1);                        //复位校准寄存器
//	  while(ADC_GetCalibrationStatus(ADC1));       //等待校准寄存器复位完成
    ADC_StartCalibration(ADC1);                        //ADC1开始校准
		//ADC_StartCalibration(ADC2);
		while (ADC_GetCalibrationStatus(ADC1) )
  {
  }
	   ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
    //while(ADC_GetCalibrationStatus(ADC1));            //等待校准完成					   
//    ADC_InjectedSequencerLengthConfig(ADC1,3);  
//    ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);          //关闭注入组转换完成中断
    hPhaseAOffset=0;
    hPhaseBOffset=0;
    hPhaseCOffset=0;	
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	  ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); 
  	ADC_InjectedSequencerLengthConfig(ADC1,3);
		ADC_InjectedChannelConfig(ADC1, PHASE_U_CURR_CHANNEL, 1, ADC_SampleTime_7Cycles5);
		ADC_InjectedChannelConfig(ADC1, PHASE_V_CURR_CHANNEL, 2, ADC_SampleTime_7Cycles5);
		ADC_InjectedChannelConfig(ADC1, PHASE_W_CURR_CHANNEL, 3, ADC_SampleTime_7Cycles5);  
	  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
    ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);
//		ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
//		ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);
     for(bIndex=16; bIndex !=0; bIndex--)
    {
        while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }  //等待注入组转换完成  
 
        //求Q1.15格式的零电流值，16个（零电流值/8）的累加，把最高符号位溢出。
				hPhaseAOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)>>3);  //注入组左对齐，数据要右移3位才是真实数据
				hPhaseBOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2)>>3);
				hPhaseCOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3)>>3);
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);                                                //清除注入组转换完成中断标志 
        ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);                                     //给一个软件触发信号，开始注入组转换
    }	
//	  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);           //ADC1注入组转换的触发信号选择,注入组转换由TIM1的TRGO触发
//    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);          
	

		/* ADC1 Injected conversions trigger is TIM1 TRGO */ 
		ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4); //trgo ´¥·¢ Trgo ÊÇÊ²Ã´
		
		 ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);
		//ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);
		/* Bus voltage protection initialization*/                            
		//ADC_AnalogWatchdogCmd(ADC1,ADC_AnalogWatchdog_SingleInjecEnable);//µçÑ¹¹ý¸ß£¬¿´ÃÅ¹·¸´Î»

		//ADC_AnalogWatchdogSingleChannelConfig(ADC1,BUS_VOLT_FDBK_CHANNEL);
		//ADC_AnalogWatchdogThresholdsConfig(ADC1, OVERVOLTAGE_THRESHOLD>>3,0x00);//ÉèÖÃ¸ßÖµ


		/* ADC1 Injected group of conversions end and Analog Watchdog interrupts
		enabling */
		ADC_ITConfig(ADC1, ADC_IT_JEOC | ADC_IT_AWD, ENABLE);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	  //4个抢先级、4个子优先级

		NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);		
  
}
 void ADC_INIT(void)
  {
    Adc_GPIO_CONFIG();
    ADC_Mode_Config();
  }
	