#include "IncludeFile.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>    // time()

volatile float ADC_input;

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

extern void ADCInit(){
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
	DMA2_Stream0->M0AR |= (uint32_t) &ADC_input;	//Memory Address
	DMA2_Stream0->NDTR = 65535;
	DMA2_Stream0->CR |= 1;	//DMA enabled
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
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;//changed from 16bit
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);

}

// PD12
extern void TIM4Init(void){
	// GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	// TIM4
	RCC->APB1ENR |= RCC_APB1Periph_TIM4;
	//Shift registers operate at 25Mhz so we use a prescaler of 2.36
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 8399; //10 khz PWM
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	//Initialize TIM4
	TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	//Start count on TIM4
	TIM_Cmd(TIM4, ENABLE);

	//PWM
	TIM_OCInitTypeDef TIM_OCStruct;
	//PWM mode 2 = Clear on compare match
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCStruct.TIM_Pulse = 4199;  //50% duty cycle
	TIM_OC1Init(TIM4, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

// PA8/9/10 are red/yellow/green
extern void Traffic_Light_Init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}


// PD7 to clear the shift registers at startup
extern void Clear_Init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

extern void Set_PD7(void){
	GPIOD->ODR ^= 0xFF;// be carefull for timer
}

extern void SPI_Bus_tx(uint16_t data){
	SPI1->DR = data;

	//wait until TXE == 1
	//while( ((SPI1->SR) & SPI_I2S_FLAG_TXE) != 1){}
}

uint32_t new_car(uint32_t traffic_value){

	//generates random number between 1 and traffic_value-1
	int value = rand() & traffic_value;

	if (value == 1){
		return value;
	}else{
		return 0;
	}
}

uint32_t generate_traffic(uint32_t traffic_value, uint32_t STATE_TRAFFIC){
	uint32_t upper_cars = (STATE_TRAFFIC)&CLEARED_TRAFFIC;
	uint32_t approaching_cars = (STATE_TRAFFIC)&APPROACHING_TRAFFIC;
	upper_cars = upper_cars << 0b1;

	if(STATE_TRAFFIC & green){
		approaching_cars = approaching_cars << 0b1;
	}

	//generate new car
	uint32_t car = new_car(traffic_value);

	STATE_TRAFFIC = (upper_cars & CLEARED_TRAFFIC) | (approaching_cars) | car;
	//STATE_TRAFFIC = (uint32_t)(0xFFFFFFFF);// test val
	return STATE_TRAFFIC;
}

float read_adc(void){

	//printf("%u\n", ADC1->DR);
	ADC_input = ADC1 -> DR;
	return ADC_input;
}
