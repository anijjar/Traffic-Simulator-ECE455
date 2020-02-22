
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include "middleware.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#define ENABLE  1
#define DISABLE 0

// Use PA1
static void ADCInit(float *ADC_destination){
	//GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;// | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//ADC
	ADC_DeInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

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
	
	// DMA - store into adc dest.
	RCC->APB2ENR |= RCC_AHB1Periph_DMA1;
	DMA2_Stream0->CR |= 0x100;	//Circ mode
	DMA2_Stream0->PAR |= (uint32_t) (&(ADC1->DR));		//Peripheral Address
	DMA2_Stream0->M0AR |= (uint32_t) ADC_destination;	//Memory Address
	DMA2_Stream0->NDTR = 65535;
	DMA2_Stream0->CR |= 1;	//DMA enabled
}

static void SPIInit(void){
	// GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//Alternate Function mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	// SPI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitTypeDef SPI_InitStruct;

	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);
	
}

extern void MiddlewareHandler(float *ADC_destination){
	//TASK 3 - ADC Conversions
	ADCInit(ADC_destination);
	ADC1->CR2 |= ADC_CR2_SWSTART;// start conversions

	// TASK 2 - Write to shift Registers
	// We will use the SPI peripheral to write 
	// one bit at a time. The shift register has
	// serial input so we must write the bit string
	// one bit at a time, while activating the clock after 
	// every write.
	SPIInit();
}

extern void SPI_Bus_tx(uint16_t *data){
	SPI_I2S_SendData(SPI1, *data);
	//wait until TXE == 1
	while( ((SPI1->SR) & SPI_I2S_FLAG_TXE) == 1){}
}
extern void SPI_Bus( int mode){
	// disable
	if(mode == 0){
		SPI_Cmd(SPI1, 0);
	}
	else
	{
		SPI_Cmd(SPI1, 1);
	}
}

// This function is not required because we are using
// the dma to write into a global variable in main.
// I want to instead write into a RTOS Stream buffer
// or message to the traffic generation task this value
// without needing to poll this method.
/*
extern float read_adc(void){
 	printf("%u\n", ADC1->DR);
 	return ADC_val;
}
*/
