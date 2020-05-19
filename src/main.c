/*
	Project: Traffic Light Simulator
	Developers: Daniel MacRae & Aman Nijjar
	Summary: Model a single lane, one-way traffic system using the FreeRTOS framework.

*/

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

/* Timer period is 1 second */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS ) /* 1 sec */

/* Length of the main queue is 1 index */
#define mainQUEUE_LENGTH					( 1 )

/* Prototypes */
static void prvSetupHardware( void );
static void Traffic_Flow_Adjustment_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );
static void vTrafficLight_Callback( xTimerHandle xTimer );

/* Queues 
		xPotResistanceQueue : contains the potentiometer resistance value
		xFlowQueue : contains the calculated flow value (heavy traffic = low)

*/
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
					"Generator_Task", 				/* Text name for the task, just to help debugging. */
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
/*
	@purpose: updates the potentiometer resistance in the 
			  xPotResistanceQueue.
	@args: NULL
*/
static void Traffic_Flow_Adjustment_Task( void *pvParameters )
{
	for( ;; )
	{
		vTaskDelay(100);
		/* Read the ADC1 DR for the potentiometer resistance */
		uint32_t adc_val = read_adc();
		/* Update the xPotResistanceQueue with adc_val right away*/
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
	/* Break the board state into an "upper" an "approaching" state. This will 
	allow the traffic past the lights to keep moving, while the traffic approaching 
	doesnt overflow. */
	uint32_t upper_cars = (STATE_TRAFFIC)&CLEARED_TRAFFIC;
	uint32_t approaching_cars = (STATE_TRAFFIC)&APPROACHING_TRAFFIC;

	/* let upper traffic continue to make room for one approaching car */
	upper_cars = upper_cars << 0b1;

	/*if the light is green, then allow approaching cars to shift left one, 
	  otherwise, if there is no car in front of the traffic light, then shift 
	  all the cars, but if there is a car, find the first empty spot and shift 
	  the cars to fill that spot. */
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

	//Using the flow value, randomly determine if a car is generated
	/* Note we are not using the True random number generator peripheral, 
	   so the same flow value with potentiometer resistance will generate 
	   the same number of cars. */
	uint32_t car = new_car(traffic_value);
	/* Create a new board state */
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

/*
	@purpose: Generates the traffic to display and handles the shifting  
			  logic. 
	@args: NULL
*/
static void Traffic_Generator_Task( void *pvParameters )
{	
	/* Initial values : start with no cars and red light*/
	uint32_t traffic_val = 3;
	int32_t flow_value = 0;
	
	/* "red" is a binary number defined in middleware.h */
	uint32_t board_state = red;
	uint32_t lights = red;

	/* Send the red state to the xBoardStateQueue immediatly*/
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
		
		/* Using the board_state and traffic_val values, the traffic variable will contain 
		   the cars currently on the road. generate_traffic may or may not return a new car. */
		uint32_t traffic = generate_traffic(traffic_val, board_state);
		
		/* Get the current light state to add to the board_state variable */
		lights = board_state & light_state;
		/* Used for debugging purposes on SWD, ignore */
		char lights_value = display_lights_debug(lights);

		/* The current LED state that will be displayed on the LEDs*/
		board_state = (lights) | traffic;

		/* increment the flow value if a new car was generated from generate_traffic 
		   otherwise, decrement. The flow_value is used by the traffic_light_state_task 
		   to increase/decrease the green/red light duration.*/
		if(traffic & 1){
			flow_value++;
		}else{
			flow_value--;
		}

		printf("Lights: %c Flow: %i Traffic: %u\n", lights_value, flow_value, traffic);

		//send flow value for traffic light state task and board state to system display task
		xQueueSend( xFlowQueue, &flow_value, 0);
		xQueueSend( xBoardStateQueue, &board_state, 0);
	}
}
/*-----------------------------------------------------------*/

/* Callback for the software timer that will change the traffic light */
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

/*
	@purpose: Updates the Traffic Light using the flow value
	@args: NULL
*/
static void Traffic_Light_State_Task( void *pvParameters )
{
	/* Initialize a timer that will update the traffic lights, periodically.*/
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
/*
	@purpose: Display the board state variable using the SPI peripheral 
	@args: NULL
*/
static void System_Display_Task( void *pvParameters ){
	uint32_t board_state = 0;
	for( ;; ){
		vTaskDelay(500);
		xQueueReceive( xBoardStateQueue, &board_state, portMAX_DELAY);
		//we only want the first 6 bits of the board upper
		uint16_t board_upper = (board_state & 0x3F0000) >> 16;
		uint16_t board_lower = (board_state & 0xFFFF);
		SPI_Bus_tx(board_upper);
		/* Give time for the SPI bus to send the data since the register is 
		   16-bits long.*/
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
	ADCInit(); /* PA1 */
	SPIInit(); /* PA5 = clock | PA7 = Output */
}
