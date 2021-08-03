#include "control.h"
s32 wAux;

Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void)
{
  Curr_Components Local_Stator_Currents;
  

  switch (bSector)
   {
   case 4:  //取得在那个扇区的时候在读取AD
   case 5: //Current on Phase C not accessible     
           // Ia = (hPhaseAOffset)-(ADC Channel 11 value)    
            wAux = (s32)(hPhaseAOffset)- (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)<<1);          
           //Saturation of Ia 
            if (wAux < S16_MIN)
            {
              Local_Stator_Currents.qI_Component1= S16_MIN;
            }  
            else  if (wAux > S16_MAX)
                  { 
                    Local_Stator_Currents.qI_Component1= S16_MAX;
                  }
                  else
                  {
                    Local_Stator_Currents.qI_Component1= wAux;
                  }
                     
           // Ib = (hPhaseBOffset)-(ADC Channel 12 value)
            wAux = (s32)(hPhaseBOffset)-((ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2))<<1);
           // Saturation of Ib
            if (wAux < S16_MIN)
            {
              Local_Stator_Currents.qI_Component2= S16_MIN;
            }  
            else  if (wAux > S16_MAX)
                  { 
                    Local_Stator_Currents.qI_Component2= S16_MAX;
                  }
                  else
                  {
                    Local_Stator_Currents.qI_Component2= wAux;
                  }
           break;
           
   case 6:
   case 1:  //Current on Phase A not accessible     
            // Ib = (hPhaseBOffset)-(ADC Channel 12 value)
            wAux = (s32)(hPhaseBOffset)-((ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1))<<1);
            //Saturation of Ib 
            if (wAux < S16_MIN)
            {
              Local_Stator_Currents.qI_Component2= S16_MIN;
            }  
            else  if (wAux > S16_MAX)
                  { 
                    Local_Stator_Currents.qI_Component2= S16_MAX;
                  }
                  else
                  {
                    Local_Stator_Currents.qI_Component2= wAux;
                  }
            // Ia = -Ic -Ib 
            wAux = ((ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2))<<1)-hPhaseCOffset-
                                            Local_Stator_Currents.qI_Component2;
            //Saturation of Ia
            if (wAux> S16_MAX)
            {
               Local_Stator_Currents.qI_Component1 = S16_MAX;
            }
            else  if (wAux <S16_MIN)
                  {
                   Local_Stator_Currents.qI_Component1 = S16_MIN;
                  }
                  else
                  {  
                    Local_Stator_Currents.qI_Component1 = wAux;
                  }
           break;
           
   case 2:
   case 3:  // Current on Phase B not accessible
            // Ia = (hPhaseAOffset)-(ADC Channel 11 value)     
            wAux = (s32)(hPhaseAOffset)-((ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1))<<1);
            //Saturation of Ia 
            if (wAux < S16_MIN)
            {
              Local_Stator_Currents.qI_Component1= S16_MIN;
            }  
            else  if (wAux > S16_MAX)
                  { 
                    Local_Stator_Currents.qI_Component1= S16_MAX;
                  }
                  else
                  {
                    Local_Stator_Currents.qI_Component1= wAux;
                  }
     
            // Ib = -Ic-Ia;
            wAux = ((ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2))<<1) - hPhaseCOffset - 
                                            Local_Stator_Currents.qI_Component1;
            // Saturation of Ib
            if (wAux> S16_MAX)
            {
              Local_Stator_Currents.qI_Component2=S16_MAX;
            }
            else  if (wAux <S16_MIN)
                  {  
                    Local_Stator_Currents.qI_Component2 = S16_MIN;
                  }
                  else  
                  {
                    Local_Stator_Currents.qI_Component2 = wAux;
                  }                     
           break;

   default:
           break;
   } 
  
  return(Local_Stator_Currents); 
}