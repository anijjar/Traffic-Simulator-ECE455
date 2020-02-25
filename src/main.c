/* Standard includes. */
#include "IncludeFile.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )// configMAX_PRIORITIES = 5

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )


/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/*-----------------------------------------------------------*/
/*							Prototypes						*/
static void prvSetupHardware( void );

static void traffic_generator_task( void *pvParameters );
static void pot_read_task( void *pvParameters );
static void led_display_task( void *pvParameters );
static void traffic_light_state_task( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;
static xQueueHandle xFlowQueue = NULL;
static xQueueHandle BoardStateQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;
/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );	/* The size of each item the queue holds. */
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue, "MainQueue" );

	xFlowQueue = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );	/* The size of each item the queue holds. */
	vQueueAddToRegistry( xFlowQueue, "FlowQueue" );


	BoardStateQueue = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );	/* The size of each item the queue holds. */
	vQueueAddToRegistry( BoardStateQueue, "BoardStateQueue" );

	/* Create the semaphore used by the FreeRTOS tick hook function and the
	event semaphore task. */
	vSemaphoreCreateBinary( xEventSemaphore );
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xEventSemaphore, "xEventSemaphore" );

	/* Create the queue receive task as described in the comments at the top
	of this	file.  http://www.freertos.org/a00125.html */
	xTaskCreate( 	traffic_generator_task,			/* The function that implements the task. */
					"traffic_gen_task", 		/* Text name for the task, just to help debugging. */
					configMINIMAL_STACK_SIZE, 		/* The size (in words) of the stack that should be created for the task. */
					NULL, 							/* A parameter that can be passed into the task.  Not used in this simple demo. */
					mainQUEUE_RECEIVE_TASK_PRIORITY,/* The priority to assign to the task.  tskIDLE_PRIORITY (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1 is the highest priority. */
					NULL );							/* Used to obtain a handle to the created task.  Not used in this simple demo, so set to NULL. */


	/* Create the queue send task in exactly the same way.  Again, this is
	described in the comments at the top of the file. */
	xTaskCreate( 	pot_read_task,
					"pot_read_task",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainQUEUE_SEND_TASK_PRIORITY,
					NULL );

	/* Create the queue send task in exactly the same way.  Again, this is
	described in the comments at the top of the file. */
	xTaskCreate( 	traffic_light_state_task,
					"light_state_task",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainQUEUE_SEND_TASK_PRIORITY,
					NULL );

	/* Create the queue send task in exactly the same way.  Again, this is
	described in the comments at the top of the file. */
	xTaskCreate( 	led_display_task,
					"led_state_task",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainQUEUE_SEND_TASK_PRIORITY,
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

static void led_display_task( void *pvParameters ){

	uint32_t board_state = 0;

	for( ;; ){
		vTaskDelay(500);
		Set_PD7();
		xQueueReceive( BoardStateQueue, &board_state, portMAX_DELAY);
		//we only want the first 4 bits of the board upper
		uint16_t board_upper_bottom = (board_state & 0xFFFF0000) >> 16;
		//board lower consists of the important 16 bits
		uint16_t board_lower = board_state & 0xFFFF;
		SPI_Bus_tx(board_upper_bottom);
		vTaskDelay(5);
		SPI_Bus_tx(board_lower);

		xQueueSend( BoardStateQueue, &board_state, portMAX_DELAY);
	}
	// Basically, we are assuming the highway is 32 cars long, but
	// we are only looking at the first 19 cars.
}

static void pot_read_task( void *pvParameters )
{

	for( ;; )
	{
//		write_LED();
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, the constant used converts ticks
		to ms.  While in the Blocked state this task will not consume any CPU
		time.  http://www.freertos.org/vtaskdelayuntil.html */
		vTaskDelay(250);

		//uint32_t adc_val = read_adc();
		//adv_val = (adc_max - adc_val)/(adc_val) --> generate relative value
		uint32_t adc_val = 3; //med-low traffic.

		/* Send to the queue - causing the queue receive task to unblock and
		increment its counter.  0 is used as the block time so the sending
		operation will not block - it shouldn't need to block as the queue
		should always be empty at this point in the code. */
		xQueueSend( xQueue, &adc_val, 0 );
	}
}

char display_lights(uint32_t lights){
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

static void traffic_generator_task( void *pvParameters )
{
	uint32_t traffic_val = 3;
	uint32_t flow_value = 0;
	uint32_t board_state = red;
	uint32_t lights = red;
	xQueueSend( BoardStateQueue, &board_state, 0);

	for( ;; )
	{
		vTaskDelay(250);
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h.  http://www.freertos.org/a00118.html */
		xQueueReceive( xQueue, &traffic_val, 0);
		xQueueReceive( xFlowQueue, &flow_value, 0);

		//Traffic value pulled off the queue, which is the value of the potentiometer.
		xQueueReceive( BoardStateQueue, &board_state, portMAX_DELAY);
		uint32_t traffic = generate_traffic(traffic_val, board_state);

		lights = board_state & light_state;
		char lights_value = display_lights(lights);

		board_state = (lights) | traffic;

		if(traffic & 1){
			//new car, increment flow.
			flow_value++;
		}else{
			flow_value--;
		}

		printf("Lights: %c Flow: %lu Traffic: %lu\n", lights_value, flow_value, traffic);

		//send flow value for traffic light state task and board state.
		xQueueSend( xFlowQueue, &flow_value, 0);
		xQueueSend( BoardStateQueue, &board_state, 0);
	}
}

/*-----------------------------------------------------------*/
static void traffic_light_state_task( void *pvParameters )
{
	uint32_t board_state = 0;
	uint32_t current_light_state = 0;
	uint32_t flow_value = 0;
	uint32_t default_delay = 2000;

	int delay = 2000;

	for( ;; )
	{
		vTaskDelay(delay);
		xQueueReceive( BoardStateQueue, &board_state, portMAX_DELAY );
		xQueueReceive( xFlowQueue, &flow_value, 0 );
		current_light_state = board_state & light_state;

		switch(current_light_state){
			case green:
				current_light_state = yellow;
				vTaskDelay(delay + (flow_value*100));
				break;
			case yellow:
				current_light_state = red;
				vTaskDelay(default_delay);
				break;
			case red:
				current_light_state = green;
				vTaskDelay(delay - (flow_value*100));
				break;
			default:
				printf("error\n");
				break;
		}

		board_state = (board_state & ALL_TRAFFIC) | current_light_state;
		char light = display_lights(current_light_state);

		xQueueSend(BoardStateQueue, &board_state, portMAX_DELAY);

	}
/*
		 Do not wait if there is nothing in queue
		xQueueReceive( xFlowQueue, &flow_value, 0 );

		//wait to get the current state of traffic
		xQueueReceive( BoardStateQueue, &board_state, portMAX_DELAY );

		current_light_state = board_state & light_state;

		char light = display_lights(current_light_state);

		switch(current_light_state){
			case red:
				if(flow_value > old_flow_value){
					//flow increasing, decrease red light duration.
					delay -= (flow_value - old_flow_value) * 100;
				}else if(flow_value < old_flow_value){
					//flow decreasing
					delay += (flow_value - old_flow_value) * 100;
				}else{
					//pass
				}
			case green:
				if(flow_value > old_flow_value){
					//flow increasing, decrease red light duration.
					delay -= (flow_value - old_flow_value) * 100;
				}else if(flow_value < old_flow_value){
					//flow decreasing
					delay += (flow_value - old_flow_value) * 100;
				}else{
					//pass
				}
		}

		old_flow_value = flow_value;
		delay -= 200;

		if(delay > 0){
			//light not finished, put the board state back
			xQueueSend(BoardStateQueue, &board_state, portMAX_DELAY);
			vTaskDelay(delay);
		}else if(delay < 0){
			//change light
				if(current_light_state == yellow){
					current_light_state = red;
				}else if(current_light_state == green){
					//turn yellow:
					flow_value = 0;
					current_light_state = yellow;
				}else if(current_light_state == red){
					//turn green:
					current_light_state = green;
				}
			}

			//give new board state.
			board_state = (board_state & ALL_TRAFFIC) | current_light_state;
			char afterlight = display_lights(current_light_state);

			xQueueSend(BoardStateQueue, &board_state, portMAX_DELAY);
			xQueueSend( xFlowQueue, &flow_value, 0);
			delay = default_delay;
		}
*/
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );
	//TASK 3 - ADC Conversions
	ADCInit();
	ADC1->CR2 |= ADC_CR2_SWSTART;// start conversions

	SPIInit();
	TIM4Init();
	Set_PD7();
	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
