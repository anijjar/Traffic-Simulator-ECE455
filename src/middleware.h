/* Standard includes. */
#include <stdint.h>

// PA1
extern void ADCInit(void);

// PA4 = NSS, PA5 = clk, PA7 = output
extern void SPIInit(void);

// PD12
extern void TIM4Init(void);

// PD7 to clear the shift registers at startup
extern void Clear_Init(void);

extern void SPI_Bus_tx(uint16_t data);

extern float read_adc(void);

extern void Clear_ShiftRegisters(void);
