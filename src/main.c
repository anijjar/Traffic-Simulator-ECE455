/* Standard includes. */
#include <stdint.h>
#include "stm32f4_discovery.h"
#include "middleware.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS ) /* 1 sec */
#define mainQUEUE_LENGTH					( 1 )

/* Prototypes */
static void prvSetupHardware( void );
static void Traffic_Flow_Adjustment_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );
static void vTrafficLight_Callback( xTimerHandle xTimer );

/* Queues */
static xQueueHandle xPotResistanceQueue = NULL;
static xQueueHandle xFlowQueue = NULL;
static xQueueHandle xBoardStateQueue = NULL;

/*-----------------------------------------------------------*/

int main(void)
{
	/* Setup the hardware peripherals */
	prvSetupHardware();

	/* @arg1: queue length | @arg2: size of one element */
	xPotResistanceQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );
	xFlowQueue = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( uint32_t ) );
	xBoardStateQueue = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( uint32_t ) );

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xPotResistanceQueue, "PotQueue" );
	vQueueAddToRegistry( xFlowQueue, "FlowQueue" );
	vQueueAddToRegistry( xBoardStateQueue, "BoardStateQueue" );

	/* Create the tasks */
	xTaskCreate( 	Traffic_Generator_Task,			/* The function that implements the task. */
					"Generator_Task", 			/* Text name for the task, just to help debugging. */
					configMINIMAL_STACK_SIZE, 		/* The size (in words) of the stack that should be created for the task. */
					NULL, 							/* A parameter that can be passed into the task. */
					configMAX_PRIORITIES,			/* The priority to assign to the task. */
					NULL );							/* Used to obtain a handle to the created task.  */

	xTaskCreate( 	Traffic_Flow_Adjustment_Task,
					"Flow_Adjustment_Task",
					configMINIMAL_STACK_SIZE,
					NULL,
					configMAX_PRIORITIES,
					NULL );

	xTaskCreate( 	Traffic_Light_State_Task,
					"Light_State_Task",
					configMINIMAL_STACK_SIZE,
					NULL,
					configMAX_PRIORITIES,
					NULL );

	xTaskCreate( 	System_Display_Task,
					"Board_State_Task",
					configMINIMAL_STACK_SIZE,
					NULL,
					configMAX_PRIORITIES,
					NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details.  http://www.freertos.org/a00111.html */
	for( ;; );
}

/*-----------------------------------------------------------*/
static void Traffic_Flow_Adjustment_Task( void *pvParameters )
{

	for( ;; )
	{
		vTaskDelay(100);
		uint32_t adc_val = read_adc();
		xQueueSend( xPotResistanceQueue, &adc_val, 0 );
	}
}
/*-----------------------------------------------------------*/
uint32_t new_car(uint32_t traffic_value){

	int value = rand() % 4096;

	if(traffic_value > 3796){
		value = rand() % 6;
		if(value == 1){
			return 1;
		}else{
			return 0;
		}
	}else{
		if (value < (4096 - traffic_value)){
			return 1;
		}else{
			return 0;
		}
	}
}

uint32_t generate_traffic(uint32_t traffic_value, uint32_t STATE_TRAFFIC){
	uint32_t upper_cars = (STATE_TRAFFIC)&CLEARED_TRAFFIC;
	uint32_t approaching_cars = (STATE_TRAFFIC)&APPROACHING_TRAFFIC;

	upper_cars = upper_cars << 0b1;

	if(STATE_TRAFFIC & green){
		approaching_cars = approaching_cars << 0b1;
	}else{
		if(approaching_cars & (1 << 7)){
			//car in the front, skip.
			//find first zero
			int i;
			for(i = 6; i > 0; i--){
				if((approaching_cars & (1 << i)) == 0){
					//shift lower i - 1 bits
					int j, temp = 0;
					for(j = 0; j < i; j++){
						temp = temp | (1 << j);
					}

					uint8_t lower = approaching_cars & temp;
					lower = lower << 1;
					uint8_t upper = (approaching_cars & (0xFF & ~temp));

					approaching_cars = upper + lower;
					break;
				}
			}
		}else{
			approaching_cars = approaching_cars << 0b1;
		}
	}

	//generate new car
	uint32_t car = new_car(traffic_value);

	STATE_TRAFFIC = (upper_cars & CLEARED_TRAFFIC) | (approaching_cars) | car;

	return STATE_TRAFFIC;
}

char display_lights_debug(uint32_t lights){
	//debugging purposes//
	switch(lights){
		case green:
			return 'g';
		case red:
			return 'r';
		case yellow:
			return 'y';
		default:
			return '?';
	}
}

static void Traffic_Generator_Task( void *pvParameters )
{
	uint32_t traffic_val = 3;
	int32_t flow_value = 0;

	uint32_t board_state = red;
	uint32_t lights = red;
	xQueueSend( xBoardStateQueue, &board_state, 0);
	vTaskDelay(50);

	for( ;; )
	{
		vTaskDelay(500);
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h.  http://www.freertos.org/a00118.html */
		xQueueReceive( xPotResistanceQueue, &traffic_val, 0);
		xQueueReceive( xFlowQueue, &flow_value, 0);

		//Traffic value pulled off the queue, which is the value of the potentiometer.
		xQueueReceive( xBoardStateQueue, &board_state, portMAX_DELAY);
		uint32_t traffic = generate_traffic(traffic_val, board_state);

		lights = board_state & light_state;
		char lights_value = display_lights_debug(lights);

		board_state = (lights) | traffic;

		if(traffic & 1){
			//new car, increment flow.
			flow_value++;
		}else{
			flow_value--;
		}

		printf("Lights: %c Flow: %i Traffic: %u\n", lights_value, flow_value, traffic);

		//send flow value for traffic light state task and board state.
		xQueueSend( xFlowQueue, &flow_value, 0);
		xQueueSend( xBoardStateQueue, &board_state, 0);
	}
}
/*-----------------------------------------------------------*/
static void vTrafficLight_Callback( xTimerHandle xTrafficLight )
{
	uint32_t board_state = 0;

	xQueueReceive( xBoardStateQueue, &board_state, portMAX_DELAY );

	uint32_t current_light_state = board_state & light_state;

	switch(current_light_state){
		case green:
			current_light_state = yellow;
			break;
		case yellow:
			current_light_state = red;
			break;
		case red:
			current_light_state = green;
			break;
		default:
			printf("light error\n");
			break;
	}

	board_state = current_light_state | (board_state & ALL_TRAFFIC);

	xQueueSend( xBoardStateQueue, &board_state, portMAX_DELAY );
}

static void Traffic_Light_State_Task( void *pvParameters )
{
	xTimerHandle xTrafficLightTimer = NULL;
	uint32_t default_delay = mainSOFTWARE_TIMER_PERIOD_MS * 4;
	xTrafficLightTimer = xTimerCreate(	"TrafficLightTimer", 	/* A text name, purely to help debugging. */
										default_delay,			/* (4s). */
										pdFALSE,				/* This is a periodic timer, so xAutoReload is set to pdTRUE. */
										( void * ) 0,			/* The ID is not used, so can be set to anything. */
										vTrafficLight_Callback	/* The callback function that switches the LED off. */
										);

	int32_t flow_value = 0;
	uint32_t board_state = 0;
	int32_t new_green_time = default_delay;
	int32_t new_red_time = default_delay;
	int delay = 100;

	xTimerStart( xTrafficLightTimer, 0);
	for( ;; )
	{
		vTaskDelay(delay);
		//set the timer period according to current light state and flow value.

		xQueueReceive( xBoardStateQueue, &board_state, portMAX_DELAY );
		xQueueReceive( xFlowQueue, &flow_value, 0 );

		uint32_t current_light_state = board_state & light_state;

		//wait until timer finished.
		 if( xTimerIsTimerActive( xTrafficLightTimer ) == pdFALSE ){
			switch(current_light_state){
				case green:
					//increase green time if flow is positive

					new_green_time = default_delay + (flow_value * 500);

					if(new_green_time > default_delay * 2){
						//if more than double, set to double.
						new_green_time = default_delay * 2;
					}else if(new_green_time < default_delay / 2){
						new_green_time = default_delay / 2;
					}

					xTimerChangePeriod(xTrafficLightTimer, new_green_time, 0);
					break;

				case yellow:
					//going to change red, set red delay
					xTimerChangePeriod(xTrafficLightTimer, default_delay/2, 0);
					break;

				case red:
					new_red_time = default_delay - (flow_value * 500);

					if(new_red_time > default_delay * 2){
						//if more than double, set to double.
						new_red_time = default_delay * 2;
					}else if(new_red_time < default_delay / 2){
						new_red_time = default_delay / 2;
					}

					//going to change green, set green delay
					xTimerChangePeriod(xTrafficLightTimer, new_red_time, 0);
					break;
				default:
					printf("light task error\n");
					break;

			}
			flow_value = 0;
		}

		xQueueSend( xBoardStateQueue, &board_state, portMAX_DELAY);
		xQueueSend( xFlowQueue, &flow_value, portMAX_DELAY);
	}
}
/*-----------------------------------------------------------*/
static void System_Display_Task( void *pvParameters ){
	uint32_t board_state = 0;
	for( ;; ){
		vTaskDelay(500);
		xQueueReceive( xBoardStateQueue, &board_state, portMAX_DELAY);
		//we only want the first 6 bits of the board upper
		uint16_t board_upper = (board_state & 0x3F0000) >> 16;
		uint16_t board_lower = (board_state & 0xFFFF);
		SPI_Bus_tx(board_upper);
		vTaskDelay(1);
		SPI_Bus_tx(board_lower);
		xQueueSend( xBoardStateQueue, &board_state, portMAX_DELAY);
	}
}
/*-----------------------------------------------------------*/
/* Setup the STM32F4 Hardware	*/
static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );
	ADCInit();/* PA1 */
	SPIInit();/* PA5 = clock | PA7 = Output */
}
