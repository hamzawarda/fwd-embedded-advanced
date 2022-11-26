/**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  Gpio.h
 *       Module:  GPIO
 *
 *  Description:  header file for General Purpose Input Output module  
 *  
 *********************************************************************************************************************/
#ifndef Gpio_H
#define Gpio_H

/**********************************************************************************************************************
 * INCLUDES
 *********************************************************************************************************************/
#include "Std_Types.h"
#include "tm4c123gh6pm.h"

/**********************************************************************************************************************
 *  GLOBAL CONSTANT MACROS
 *********************************************************************************************************************/


/**********************************************************************************************************************
 *  GLOBAL FUNCTION MACROS
 *********************************************************************************************************************/


/**********************************************************************************************************************
 *  GLOBAL DATA TYPES AND STRUCTURES
 *********************************************************************************************************************/


/**********************************************************************************************************************
 *  GLOBAL DATA PROTOTYPES
 *********************************************************************************************************************/

 
/**********************************************************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/
 
/******************************************************************************
* \Syntax          : void GPio_F_Init(void)                                      
* \Description     : initialize GPIO port F Module by parsing the Configuration 
*                    into GPIO F registers                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : None                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
void Gpio_F_Init(void);

/******************************************************************************
* \Syntax          : void GPio_Pf1_On(void)                                      
* \Description     : set GPIO portf Pin 1 with high                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : None                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
void Gpio_Pf1_On(void);

/******************************************************************************
* \Syntax          : void GPio_Pf1_Off(void)                                      
* \Description     : set GPIO portf Pin 1 with Low                                    
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : None                     
* \Parameters (out): None                                                      
* \Return value:   : None
*******************************************************************************/
void Gpio_Pf1_Off(void);
 
/******************************************************************************
* \Syntax          : uint32 GPio_Pf_Read(uint32)                                      
* \Description     : read a pin value on port f                                   
*                                                                             
* \Sync\Async      : Synchronous                                               
* \Reentrancy      : Non Reentrant                                             
* \Parameters (in) : pin number                     
* \Parameters (out): pin value                                                
* \Return value:   : None
*******************************************************************************/
uint32 Gpio_Pf_Read(uint32 pin);
 
#endif  /* Gpio_H */

/**********************************************************************************************************************
 *  END OF FILE: Gpio.h
 *********************************************************************************************************************/
