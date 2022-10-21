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

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "queue.h"

int task_1_start_time = 0, task_1_end_time = 0, task_1_TotalTime=0;														
int task_2_start_time = 0, task_2_end_time = 0, task_2_TotalTime=0;														
int task_3_start_time = 0, task_3_end_time = 0, task_3_TotalTime=0;														
int task_4_start_time = 0, task_4_end_time = 0, task_4_TotalTime=0;														
int task_5_start_time = 0, task_5_end_time = 0, task_5_TotalTime=0;														
int task_6_start_time = 0, task_6_end_time = 0, task_6_TotalTime=0;				
int system_time = 0;														
int CPU_load = 0;

struct AMessage
{
    char ucMessageID;
    char ucData[ 20 ];
};
QueueHandle_t xStructQueue = NULL;

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
}
TaskHandle_t xButton_1_Monitor = NULL;
pinState_t myPrevstate=PIN_IS_HIGH;
void Button_1_Monitor(void* pvParameters)
{
	struct AMessage pxfall={'h',"B1: Falling Edge\n"};
	struct AMessage pxrise={'h',"B1: Rising Edge\n"};

	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;

     // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	while(1)
	{
			myPrevstate=GPIO_read(PORT_0,PIN1);
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	
	if( GPIO_read(PORT_0,PIN1) != myPrevstate)
	{
		if(myPrevstate==PIN_IS_HIGH)
		{
			xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &pxfall,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 0 );
		}
		else
					xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &pxrise,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 0 );
	}
	
	}
		
}

TaskHandle_t xButton_2_Monitor = NULL;
pinState_t myPrevstate2=PIN_IS_HIGH;
void Button_2_Monitor(void* pvParameters)
{
	struct AMessage pxfall={'k',"B2: Falling Edge\n"};
	struct AMessage pxrise={'k',"B2: Rising Edge\n"};
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;

     // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	while(1)
	{
			myPrevstate2=GPIO_read(PORT_0,PIN2);
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	
	if( GPIO_read(PORT_0,PIN2) != myPrevstate2)
	{
		if(myPrevstate2==PIN_IS_HIGH)
		{
			xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &pxfall,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 10 );
		}
		else
					xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &pxrise,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 10 );
	}
		
}
}

//--------------------------------------------------------------------------------------------------------------
TaskHandle_t xPeriodic_Transmitter = NULL;
void Periodic_Transmitter(void* pvParameters)
{
	struct AMessage pxPerString={'p',"periodic string\n"};
	

	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100;

     // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &pxPerString,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 10 );
	}

	
		
}
//--------------------------------------------------------------------------------------------------------------
TaskHandle_t xUart_Receiver= NULL;
void Uart_Receiver(void* pvParameters)
{
	struct AMessage xRxedStructure;
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100;

     // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		xQueueReceive( xStructQueue,
                         &( xRxedStructure ),
                         portMAX_DELAY );
		vSerialPutString(xRxedStructure.ucData,sizeof(xRxedStructure.ucData));
	}
}
//--------------------------------------------------------------------------------------------------------------
TaskHandle_t xLoad_1_Simulation= NULL;
void Load_1_Simulation(void* pvParameters)
{
	int i=0;
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;

     // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		for( i=0;i<33500;i++)
		{i=i;}
	}
}
//--------------------------------------------------------------------------------------------------------------
TaskHandle_t xLoad_2_Simulation= NULL;
void Load_2_Simulation(void* pvParameters)
{
	int i=0;
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100;

     // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		for( i=0;i<79500;i++)
		{i=i;}
	}
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	xStructQueue=xQueueCreate( 10, sizeof(struct AMessage) );
	
    /* Create Tasks here */
	 xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1_Monitor",          /* Text name for the task. */
                    133,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &xButton_1_Monitor
										,50
										);      /* Used to pass out the created task's handle. */

	xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2_Monitor",          /* Text name for the task. */
                    133,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &xButton_2_Monitor
										,50
										);      /* Used to pass out the created task's handle. */
 /* Create Tasks here */
	 xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Periodic_Transmitter",          /* Text name for the task. */
                    133,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &xPeriodic_Transmitter
										,100
								);      /* Used to pass out the created task's handle. */
	 xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Uart_Receiver",          /* Text name for the task. */
                    133,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &xUart_Receiver
										,20
								);      /* Used to pass out the created task's handle. */
    /* Create the task, storing the handle. */
/* Create Tasks here */
xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load_1_Simulation",          /* Text name for the task. */
                    133,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &xLoad_1_Simulation
										,10
								);     
/* Create Tasks here */
xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load_2_Simulation",          /* Text name for the task. */
                    133,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &xLoad_2_Simulation
										,100
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
/*-----------------------------------------------------------*/


