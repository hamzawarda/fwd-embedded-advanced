/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define usStackDepth 90
#define Delay_1ms 12000

/* Port0 pins mapping */
#define Button_1 PIN0
#define Button_2 PIN1
#define Tick_Led PIN2
#define Button_1_Led PIN3
#define Button_2_Led PIN4
#define Periodic_Transmitter_Led PIN5
#define Uart_Receiver_Led PIN6
#define Load_1_Simulation_Led PIN7
#define Load_2_Simulation_Led PIN8
#define Idle_Led PIN9

enum message
{
	Empty,
	Button_1_Rising,
	Button_1_Faling,
	Button_2_Rising,
	Button_2_Faling,
	Periodic_Tx
};

enum message Message;

QueueHandle_t xQueueUart;
TaskHandle_t Button_1_Monitor_Handle = NULL;
TaskHandle_t Button_2_Monitor_Handle = NULL;
TaskHandle_t Periodic_Transmitter_Handle = NULL;
TaskHandle_t Uart_Receiver_Handle = NULL;
TaskHandle_t Load_1_Simulation_Handle = NULL;
TaskHandle_t Load_2_Simulation_Handle = NULL;

/*-----------------------------------------------------------
 *Prototypes
-----------------------------------------------------------*/
void Button_1_Monitor_Function(void * pvParameters);
void Button_2_Monitor_Function(void * pvParameters);
void Periodic_Transmitter_Function(void * pvParameters);
void Uart_Receiver_Function(void * pvParameters);
void Load_1_Simulation_Function(void * pvParameters);
void Load_2_Simulation_Function(void * pvParameters);

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	/* Create Queue for messages */
	xQueueUart = xQueueCreate( 10, sizeof( enum message ) );
	
	/* Create Tasks here */
	xTaskPeriodCreate(
		Button_1_Monitor_Function,
    "B1Mntr",
    usStackDepth,
    (void *)1,
    1,
    &Button_1_Monitor_Handle,
		(TickType_t) 50
    );
		
	xTaskPeriodCreate(
		Button_2_Monitor_Function,
    "B2Mntr",
    usStackDepth,
    (void *)1,
    1,
    &Button_2_Monitor_Handle,
		(TickType_t) 50
    );

	xTaskPeriodCreate(
		Periodic_Transmitter_Function,
    "PrTx",
    usStackDepth,
    (void *)1,
    1,
    &Periodic_Transmitter_Handle,
		(TickType_t) 100
    );

	xTaskPeriodCreate(
		Uart_Receiver_Function,
    "UrtRx",
    usStackDepth,
    (void *)1,
    1,
    &Uart_Receiver_Handle,
		(TickType_t) 20
    );

	xTaskPeriodCreate(
		Load_1_Simulation_Function,
    "Load_1",
    usStackDepth,
    (void *)1,
    1,
    &Load_1_Simulation_Handle,
		(TickType_t) 10
    );

	xTaskPeriodCreate(
		Load_2_Simulation_Function,
    "Load_2",
    usStackDepth,
    (void *)1,
    1,
    &Load_2_Simulation_Handle,
		(TickType_t) 100
    );		

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}

/* Task-1 */
void Button_1_Monitor_Function(void * pvParameters)
{
	/* Setup delay time */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;
	
	/* monitor rising and falling edge on button 1 and send this event to the consumer task. */
	pinState_t Previous = PIN_IS_LOW, Current = PIN_IS_LOW;
	vTaskSetApplicationTaskTag( NULL, ( TaskHookFunction_t  ) 1 );
	
	
	/* Initialise the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		Current = GPIO_read(PORT_0, Button_1);
		if(Current != Previous) /* rising - faling edge condition */
		{
			if(Current == PIN_IS_HIGH)
			{
				Message= Button_1_Rising;
				xQueueSend( xQueueUart, &Message, portMAX_DELAY);
			}
			else
			{
				Message= Button_1_Faling;
				xQueueSend( xQueueUart, &Message, portMAX_DELAY);
			}
			Previous = Current;
		}
		/* Wait for the next cycle. */
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/* Task-2 */
void Button_2_Monitor_Function(void * pvParameters)
{
	/* Setup delay time */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;
	
	/* monitor rising and falling edge on button 2 and send this event to the consumer task. */
	pinState_t Previous = PIN_IS_LOW, Current = PIN_IS_LOW;
	vTaskSetApplicationTaskTag( NULL, ( TaskHookFunction_t ) 2 );
		
	/* Initialise the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		Current = GPIO_read(PORT_0, Button_2);
		if(Current != Previous) /* rising - faling edge condition */
		{
			if(Current == PIN_IS_HIGH)
			{
				Message= Button_2_Rising;
				xQueueSend( xQueueUart, &Message, portMAX_DELAY);
			}
			else
			{
				Message= Button_2_Faling;
				xQueueSend( xQueueUart, &Message, portMAX_DELAY);
			}
			Previous = Current;
		}	
		/* Wait for the next cycle. */
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/* Task-3 */
void Periodic_Transmitter_Function(void * pvParameters)
{
	/* Setup delay time */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	
	/* send preiodic string */
	vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t ) 3 );
		
	/* Initialise the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();	
	for( ;; )
	{
		Message= Periodic_Tx;
		xQueueSend( xQueueUart, &Message, portMAX_DELAY);
		/* Wait for the next cycle. */
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/* Task-4 */
void Uart_Receiver_Function(void * pvParameters)
{
	/* Setup delay time */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	BaseType_t Ret = pdFALSE;
	
	/* write on UART any received string */
	vTaskSetApplicationTaskTag( NULL, ( TaskHookFunction_t ) 4 );
	
	/*enum message Message;*/
	/*BaseType_t Ret;*/
	/* Initialise the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();	
	for( ;; )
	{
		Ret= xQueueReceive(xQueueUart, &Message, 0);
		if(Ret == pdTRUE)
		{
			if(Message == Button_1_Rising)
				vSerialPutString("Button 1 rising\n",16);
			else if(Message == Button_1_Faling)
				vSerialPutString("Button 1 faling\n",16);
			else if(Message == Button_2_Rising)
				vSerialPutString("Button 2 rising\n",16);
			else if(Message == Button_2_Faling)
				vSerialPutString("Button 2 faling\n",16);
			else if(Message == Periodic_Tx)
				vSerialPutString("Periodic Message\n",17);
		}
		/* Wait for the next cycle. */
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/* Task-5 Load-1 */
void Load_1_Simulation_Function(void * pvParameters)
{
	/* Setup delay time */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;
	
	/* Execution time: 5ms */
	uint32_t Delay = 0;
	uint32_t Delay_end = 5*Delay_1ms;
	vTaskSetApplicationTaskTag( NULL, ( TaskHookFunction_t ) 5 );
	
	/* Initialise the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();	
	for( ;; )
	{
		for(Delay=0; Delay<Delay_end; Delay++){};
		/* Wait for the next cycle. */
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/* Task-6 Load-2 */
void Load_2_Simulation_Function(void * pvParameters)
{
	/* Setup delay time */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	/* Execution time: 12ms */
	uint32_t Delay = 0;
	uint32_t Delay_end = 12*Delay_1ms;
	vTaskSetApplicationTaskTag( NULL, ( TaskHookFunction_t ) 6 );
	

	/* Initialise the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();	
	for( ;; )
	{
		for(Delay=0; Delay<Delay_end; Delay++){};
		/* Wait for the next cycle. */
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/* Tick Hook Implementation */
void vApplicationTickHook()
{
	GPIO_write(PORT_0, Tick_Led, PIN_IS_HIGH);
	GPIO_write(PORT_0, Tick_Led, PIN_IS_LOW);
}

/*-----------------------------------------------------------*/


