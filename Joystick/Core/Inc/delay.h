/**
  * @file           delay.h
  * @details        This file provides definition of function's prototypes for make delay.
  * @author         Shima Nosrati
  * @Date           1400.07.03
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DELAY_H
  #define _DELAY_H
    
  /* Includes ------------------------------------------------------------------*/ 
  #include "stm32f0xx_hal.h"
  
  /* Exported Functions --------------------------------------------------------*/
  void delay_us (uint32_t DelayUs);
  void delay_ms (uint16_t DelayMs);
  
#endif    /* _DELAY_H */
