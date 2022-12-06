/**********************************************************************************************************************
 *  FILE DESCRIPTION
 *  -----------------------------------------------------------------------------------------------------------------*/
/**        \file  main.c
 *        \brief  main file
 *
 *      \details  user systick ISR, and main function
 *               
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "Std_Types.h"
#include "Gpio.h"
#include "Timer.h"
#include "Interrupt.h"

/* LED state values. */
#define ON_State 1
#define OFF_State 0

#define MAX_Time 4500000

/* Input switches number in Port F */
#define SW0 0 /* increase on time */
#define SW1 4 /* decrease on time */
/**********************************************************************************************************************
*  LOCAL MACROS CONSTANT\FUNCTION
*********************************************************************************************************************/	

/**********************************************************************************************************************
 *  LOCAL DATA 
 *********************************************************************************************************************/
uint32 State= OFF_State;
/* Delay time in Micro seconds */
volatile uint32 ON_Time = 4000000;
volatile uint32 OFF_Time = 1000000;

/**********************************************************************************************************************
 *  GLOBAL DATA
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL FUNCTIONS
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  GLOBAL FUNCTIONS
 *********************************************************************************************************************/

/******************************************************************************
* \Syntax          : void TimerIsr(void)                                      
* \Description     : Systick User ISR                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : None                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
void TimerIsr(void){
	if(State == OFF_State){
		Gpio_Pf1_On();
		State= ON_State;
		SysTick_Run(ON_Time);
	}
	else{
		Gpio_Pf1_Off();
		State= OFF_State;
		SysTick_Run(OFF_Time);
	}
}

/******************************************************************************
* \Syntax          : int main(void)                                      
* \Description     : main function contains GPIO, Timer, and Interrupt Initializations                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : None                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
int main (void){
	/* collect switches current and previous states. */
	uint32 current0=0, current1=0, previous0=0, previous1= 0;
	
	Gpio_F_Init();
	SysTick_Init();
	SysTick_Callback(TimerIsr);
	SysTick_Run(OFF_Time);
	while(1){
		
		/* Read a switch if pressed */
		current0 = Gpio_Pf_Read(SW0);
		current1 = Gpio_Pf_Read(SW1);
		
		
		if(current0 != previous0){
			/* state change detected, work on releas only */
			if(current0 == 0){
				/* check limits */
				if(ON_Time < MAX_Time){
					/* ON++, OFF-- */
					ON_Time+=500000;
					OFF_Time-=500000;					
				}
			}
			previous0= current0;
		}
		
		if(current1 != previous1){
			/* state change detected, work on releas only */
			if(current1 == 0){
				/* check limits */
				if(OFF_Time<MAX_Time){
					/* OFF++, ON-- */
					OFF_Time+=500000;
					ON_Time-=500000;					
				}
			}
			previous1= current1;
		}
		
	}
	return 0;
}

/**********************************************************************************************************************
 *  END OF FILE: main.c
 *********************************************************************************************************************/
