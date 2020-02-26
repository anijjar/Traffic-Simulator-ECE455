#ifndef __MIDDLEWARE_H__
#define __MIDDLEWARE_H__

#define ALL_TRAFFIC 		0b0001111111111111111111
#define CLEARED_TRAFFIC 	0b0001111111111100000000
#define APPROACHING_TRAFFIC 0b0000000000000011111111
#define light_state 		0b1110000000000000000000
#define  green 				0b1000000000000000000000
#define yellow 				0b0100000000000000000000
#define    red 				0b0010000000000000000000

float read_adc(void);

// PA1
extern void ADCInit(void);

// PA4 = NSS, PA5 = clk, PA7 = output
extern void SPIInit(void);

extern void SPI_Bus_tx(uint16_t data);

#endif /* __MIDDLEWARE_H__ */
