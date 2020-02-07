/* Standard includes. */
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

//void write_LED(void){
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//
//    GPIO_InitTypeDef GPIO_InitStruct;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//    //void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
//	GPIO_ToggleBits(GPIOA, GPIO_Pin_2);
//    while(1){}
//}

void ADCInit(void){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_Cmd(ADC1, ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
    //while((ADC1->ISR & ADC_ISR_ADRDY) == ADC_ISR_ADRDY){};
}

int read_adc(void){

	//"Pin Pa1"
	ADC_SoftwareStartConv(ADC1);
	//wait
	while(!(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))){};
	//ADC1->ISR |= ADC_ISR_EOC;
	float value = ADC_GetConversionValue(ADC1);
	//reset flag
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	printf("Adc %i\n", value);

	return 0;
}
