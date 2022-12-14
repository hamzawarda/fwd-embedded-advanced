/**********************************************************************************************************************
 *  FILE DESCRIPTION
 *  -----------------------------------------------------------------------------------------------------------------*/
/**        \file  Timer.c
 *        \brief  System timer
 *
 *      \details  The Driver Configure system timer
 *               
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "Std_Types.h"
#include "tm4c123gh6pm.h"
#include "Timer.h"

/**********************************************************************************************************************
*  LOCAL MACROS CONSTANT\FUNCTION
*********************************************************************************************************************/	

/**********************************************************************************************************************
 *  LOCAL DATA 
 *********************************************************************************************************************/

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
* \Syntax          : void SysTick_Init(void)                                      
* \Description     : initialize system timer Module by connecting systick ISR                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : None                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
void SysTick_Init(void)
{
	
	/*TODO : Put the reload value*/
  NVIC_ST_RELOAD_R = 0xFFFFFF; 

	/*TODO : Configure SysTick -systim clock -Interrupt enabled, continues mood. */
	NVIC_ST_CTRL_R = 0b111u;

}

/******************************************************************************
* \Syntax          : void SysTick_Run(uint32)                                      
* \Description     : start system timer Module by parsing the Configuration 
*                    into related registers, and start counting.                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : Delay time in microseconds                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
void SysTick_Run(uint32 Dmsec)
{
		
	/*TODO : Put the reload value*/
  NVIC_ST_RELOAD_R = 16* Dmsec; 

  /*TODO : Clear systik timer*/  
	NVIC_ST_CURRENT_R =0;
	
}

/**********************************************************************************************************************
 *  END OF FILE: Timer.c
 *********************************************************************************************************************/
