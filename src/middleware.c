/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>    // time()
#include "middleware.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

//Use PA1
extern void ADCInit(){
	//GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//ADC1
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

	ADC1->CR2 |= ADC_CR2_SWSTART;// start conversions
}

// PA4 = NSS, PA5 = clk, PA7 = output
extern void SPIInit(void){
	// GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//Alternate Function mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// SPI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitTypeDef SPI_InitStruct;

	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);

}

extern void SPI_Bus_tx(uint16_t data){
	SPI1->DR = data;

	//wait until TXE == 1
	//while( ((SPI1->SR) & SPI_I2S_FLAG_TXE) != 1){}
}

float read_adc(void){
	return ADC1->DR;
}
