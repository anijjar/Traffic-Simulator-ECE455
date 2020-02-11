
#include "IncludeFile.h"

volatile float ADC_val;
//Use PA1
void ADCInit(void){
	//GPIO stuff
	RCC->AHB1ENR |= RCC_AHB1Periph_GPIOA;

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//analog mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; //max speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; //output type push pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//ADC stuff
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
	ADC1->CR1 |= ((uint32_t)0x04000000); //8-bit res
	ADC1->CR2 |= ADC_CR2_ADON;//adc on


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
	DMAInit();
	ADCInit();

	ADC1->CR2 |= ADC_CR2_SWSTART;// start conversions
}



float read_adc(void){
	printf("ADC %0.6f\n", ADC_val);

	return ADC_val;
}
