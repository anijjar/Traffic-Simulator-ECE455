
#include "IncludeFile.h"

// Use PA1
static void ADCInit(void){
	//GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_StructInit(&GPIO_InitStruct); //default parameters
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;// | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;// no pullup/pulldown
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//ADC
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

static void DMAInit(float *ADC_destination){
	RCC->APB2ENR |= RCC_AHB1Periph_DMA1;
	DMA2_Stream0->CR |= 0x100;	//Circ mode
	DMA2_Stream0->PAR |= (uint32_t) (&(ADC1->DR));		//Peripheral Address
	DMA2_Stream0->M0AR |= (uint32_t) ADC_destination;	//Memory Address
	DMA2_Stream0->NDTR = 65535;
	DMA2_Stream0->CR |= 1;	//DMA enabled
}
// PD12 is ShiftRegister Clock
// static void TIM4Init(void){
// 	// GPIO
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

// 	GPIO_InitTypeDef GPIO_InitStruct;
// 	/* Alternating functions for pins */
// 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
// 	/* Set pins */
// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
// 	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//     GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//     GPIO_Init(GPIOD, &GPIO_InitStruct);
	
// 	// TIM4
// 	RCC->APB1ENR |= RCC_APB1Periph_TIM4;
// 	/*Shift registers operate at 25Mhz so we use a prescaler of 2.36*/
// 	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
// 	TIM_BaseStruct.TIM_Prescaler = 0;
// 	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
// 	TIM_BaseStruct.TIM_Period = 8399; /*10 khz PWM*/
// 	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
//     TIM_BaseStruct.TIM_RepetitionCounter = 0;
//     /* Initialize TIM4 */
//     TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
//     /* Start count on TIM4 */
//     TIM_Cmd(TIM4, ENABLE);
// }
// static void TIM4PWM_Init(void){
// 	TIM_OCInitTypeDef TIM_OCStruct;
// 	/* Common settings */
    
//     /* PWM mode 2 = Clear on compare match */
//     TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
//     TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
//     TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

//     TIM_OCStruct.TIM_Pulse = 4199; /* 50% duty cycle */
//     TIM_OC1Init(TIM4, &TIM_OCStruct);
//     TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
// }

// // PD11 is ShiftRegister Clear
// static void ShiftRegisterClearInit(void){
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

// 	GPIO_InitTypeDef GPIO_InitStruct;
// 	/* Set pins */
// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
// 	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//     GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//     GPIO_Init(GPIOD, &GPIO_InitStruct);
// }

// static void ShiftRegisterWriteInit(void){
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

// 	GPIO_InitTypeDef GPIO_InitStruct;
// 	/* Set pins */
// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
// 	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//     GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//     GPIO_Init(GPIOD, &GPIO_InitStruct);
// }

extern void MiddlewareHandler(float *ADC_destination){
	//TASK 3
	ADCInit();
	DMAInit(ADC_destination);

	// TASK 2
	// ShiftRegisterClearInit();
	// TIM4Init();
	// TIM4PWM_Init();
	// ShiftRegisterWriteInit();

	ADC1->CR2 |= ADC_CR2_SWSTART;// start conversions
}

// extern void Clear_ShiftRegisters(void){
// 	GPIOD->ODR ^= GPIO_Pin_11;
// }
// extern void Write_ShiftRegisters(void){
// 	GPIOD->ODR ^= GPIO_Pin_13;
// }
// extern float read_adc(void){

// 	printf("%u\n", ADC1->DR);

// 	return ADC_val;
// }
