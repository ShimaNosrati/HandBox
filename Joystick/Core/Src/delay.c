/**
  * @file           delay.c
  * @details        This file provides delay's functions definition.
  * @author         Shima Nosrati
  * @Date           1400.07.03
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        
  */

/* Includes ------------------------------------------------------------------*/
#include "delay.h"

/* Exported Functions --------------------------------------------------------*/
/**
  * @brief   This function provides accurate delay (in microseconds) based on variable decremented.
  * @param   DelayUs specifies the delay time length, in microseconds.
  * @returns None
  * @note    None
  */
void delay_us (uint32_t DelayUs)
{
  /**
    * @var   uint32_t Num
    * @brief This variable holds the value of delay time length
    */
  uint32_t Num;
  
  /**
    * @var   uint8_t Time
    * @brief This variable holds the value of counting that each 11 counting make 1us delay
    */
  uint8_t Time;

  for (Num = 0; Num < DelayUs; Num++)
  {
    Time = 11;
    
    while (Time != 0)
    {
      Time--;
    }
  }
}

/**
  * @brief   This function provides accurate delay (in milliseconds) based on delay_us function.
  * @param   DelayMs specifies the delay time length, in milliseconds.
  * @returns None
  * @note    None
  */
void delay_ms (uint16_t DelayMs)
{
  /**
    * @var   uint16_t Num
    * @brief This variable holds the value of delay time length
    */
  uint16_t Num;
  
  for (Num = 0; Num < DelayMs; Num++)
  {
    delay_us(1000);
  }
}
