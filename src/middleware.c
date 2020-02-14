
#include "IncludeFile.h"

volatile float ADC_val;

//Use PA1
void GPIO_Output(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	GPIOA->CRL |=
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void ADCInit(void){
	//GPIO stuff

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	GPIOA->CRL |=
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit();

	ADC_InitTypeDef ADC_InitStruct;

	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;
	ADC_InitStruct.ADC_ScanConvMode=DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 0x01;

	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_144Cycles);

}

void DMAInit(void){
	RCC->APB2ENR |= RCC_AHB1Periph_DMA1;
	DMA2_Stream0->CR |= 0x100;	//Circ mode
	DMA2_Stream0->PAR |= (uint32_t) (&(ADC1->DR));		//Peripheral Address
	DMA2_Stream0->M0AR |= (uint32_t) ADC_val;			//Memory Address
	DMA2_Stream0->NDTR = 65535;
	DMA2_Stream0->CR |= 1;	//DMA enabled

}

void MiddlewareHandler(void){
	ADCInit();
	DMAInit();

	ADC1->CR2 |= ADC_CR2_SWSTART;// start conversions
}



float read_adc(void){

	printf("%u\n", ADC1->DR);

	return ADC_val;
}
