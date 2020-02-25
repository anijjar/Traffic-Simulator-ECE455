#define CLEARED_TRAFFIC 0b1111111111100000000
#define APPROACHING_TRAFFIC 0b11111111

#define ALL_TRAFFIC 0b1111111111111111111

#define light_state 0b1110000000000000000000
#define  green 0b1000000000000000000000
#define yellow 0b0100000000000000000000
#define    red 0b0010000000000000000000

void MiddlewareHandler(void);

float read_adc(void);

uint32_t generate_traffic(uint32_t, uint32_t);

uint32_t new_car(uint32_t);

// PA1
extern void ADCInit(void);

// PA4 = NSS, PA5 = clk, PA7 = output
extern void SPIInit(void);

// PD7
extern void Set_PD7(void);

extern void SPI_Bus_tx(uint16_t data);


extern void Clear_ShiftRegisters(void);
