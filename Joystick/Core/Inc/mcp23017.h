/**
  * @file           mcp2317.h
  * @details        This file provides definitions of MCP23017's Ports, Address, I/O Direction,
  *                 Input Polarity, Pull_up Resistor and control registers from its datasheet
  *                 and defines MCP23017 handle Structure.
  * @author         Shima Nosrati
  * @Date           1400.06.17
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MCP23017_H
  #define _MCP23017_H
    
  /* Includes ------------------------------------------------------------------*/ 
  #include "stm32f0xx_hal.h"

  /**
    * @defgroup MCP23017 device ports
    * @brief    Define i2c device slave I/O ports
    * @{
    */
  #define MCP23017_PORTA			0x00
  #define MCP23017_PORTB			0x01
  /**
    * @}
    */
  
  /**
    * @defgroup MCP23017 device address
    * @brief    Define i2c device slave address (A0-A2)
    *           Refer to 3.3.1 section of datasheet
    * @{
    */
  #define MCP23017_ADDRESS_1		0x20       /* A0 Low, A1 Low, A2 Low    */
  #define MCP23017_ADDRESS_2		0x21       /* A0 Low, A1 Low, A2 High   */
  #define MCP23017_ADDRESS_3		0x22       /* A0 Low, A1 High, A2 Low   */
  #define MCP23017_ADDRESS_4		0x23       /* A0 Low, A1 High, A2 High  */
  #define MCP23017_ADDRESS_5		0x24       /* A0 High, A1 Low, A2 Low   */
  #define MCP23017_ADDRESS_6		0x25       /* A0 High, A1 Low, A2 High  */
  #define MCP23017_ADDRESS_7		0x26       /* A0 High, A1 High, A2 Low  */
  #define MCP23017_ADDRESS_8		0x27       /* A0 High, A1 High, A2 High */
  /**
    * @}
    */
    
  /**
    * @defgroup MCP23017 device I/O direction
    * @brief    Define i2c device slave I/O direction
    *           Default state: MCP23017_IODIR_ALL_INPUT
    *           Refer to 3.5.1 section of datasheet
    * @{
    */
  #define MCP23017_IODIR_ALL_OUTPUT	0x00
  #define MCP23017_IODIR_ALL_INPUT	0xFF
  #define MCP23017_IODIR_IO0_INPUT	0x01
  #define MCP23017_IODIR_IO1_INPUT	0x02
  #define MCP23017_IODIR_IO2_INPUT	0x04
  #define MCP23017_IODIR_IO3_INPUT	0x08
  #define MCP23017_IODIR_IO4_INPUT	0x10
  #define MCP23017_IODIR_IO5_INPUT	0x20
  #define MCP23017_IODIR_IO6_INPUT	0x40
  #define MCP23017_IODIR_IO7_INPUT	0x80
  /**
    * @}
    */  

  /**
    * @defgroup MCP23017 device input polarity
    * @brief    Define i2c device slave input polarity of I/O
    *           Default state: MCP23017_IPOL_ALL_NORMAL
    *           Refer to 3.5.2 secyion of datasheet
    * @{
    */
  #define MCP23017_IPOL_ALL_NORMAL	  0x00
  #define MCP23017_IPOL_ALL_INVERTED	0xFF
  #define MCP23017_IPOL_IO0_INVERTED	0x01
  #define MCP23017_IPOL_IO1_INVERTED	0x02
  #define MCP23017_IPOL_IO2_INVERTED	0x04
  #define MCP23017_IPOL_IO3_INVERTED	0x08
  #define MCP23017_IPOL_IO4_INVERTED	0x10
  #define MCP23017_IPOL_IO5_INVERTED	0x20
  #define MCP23017_IPOL_IO6_INVERTED	0x40
  #define MCP23017_IPOL_IO7_INVERTED	0x80
  /**
    * @}
    */

  /* Pull-Up Resistor--------------------------------------------------------------*/
  /**
    * @defgroup MCP23017 device pull-up resistor
    * @brief    Define situation of i2c device slave pull-up resisters 
    *           when pin configured as an input
    *           Default state: MCP23017_GPPU_ALL_DISABLED
    *           Refer to 3.5.7 secyion of datasheet
    * @{
    */
  #define MCP23017_GPPU_ALL_DISABLED	0x00
  #define MCP23017_GPPU_ALL_ENABLED	  0xFF
  #define MCP23017_GPPU_IO0_ENABLED	  0x01
  #define MCP23017_GPPU_IO1_ENABLED	  0x02
  #define MCP23017_GPPU_IO2_ENABLED	  0x04
  #define MCP23017_GPPU_IO3_ENABLED	  0x08
  #define MCP23017_GPPU_IO4_ENABLED	  0x10
  #define MCP23017_GPPU_IO5_ENABLED	  0x20
  #define MCP23017_GPPU_IO6_ENABLED	  0x40
  #define MCP23017_GPPU_IO7_ENABLED	  0x80
  /**
    * @}
    */

   /**
    * @defgroup MCP23017 device control registers
    * @brief    Define i2c device slave Control Registers
                Refer to 3-5 table of datasheet
    * @{
    */
  #define MCP23017_REG_IODIRA		0x00
  #define MCP23017_REG_IODIRB		0x01
  #define MCP23017_REG_IPOLA		0x02
  #define MCP23017_REG_IPOLB		0x03
  #define MCP23017_REG_GPINTENA	0x04
  #define MCP23017_REG_GPINTENB	0x05
  #define MCP23017_REG_DEFVALA	0x06
  #define MCP23017_REG_DEFVALB	0x07
  #define MCP23017_REG_INTCONA	0x08
  #define MCP23017_REG_INTCONB	0x09
  #define MCP23017_REG_GPPUA		0x0C
  #define MCP23017_REG_GPPUB		0x0D
  #define MCP23017_REG_INTFA		0x0E
  #define MCP23017_REG_INTFB		0x0F
  #define MCP23017_REG_INTCAPA	0x10
  #define MCP23017_REG_INTCAPB	0x11
  #define MCP23017_REG_GPIOA		0x12
  #define MCP23017_REG_GPIOB		0x13
  #define MCP23017_REG_OLATA		0x14
  #define MCP23017_REG_OLATB		0x15
  /**
    * @}
    */
  
  /**
    * @def      MCP23017_I2C_TIMEOUT
    * @brief    Define value of i2c timeout
    */
  #define MCP23017_I2C_TIMEOUT		      10
  
  /* Exported Types ------------------------------------------------------------*/
  /**
    * @struct MCP23017_Handle_t
    * @brief  contains value and parameters of MCP23017 
    */
  typedef struct
  {
    I2C_HandleTypeDef *	hI2c;   /**< i2c peripheral handle */
    uint16_t Addr;              /**< i2c device slave address */
    uint8_t	Gpio[2];            /**< value of I/O ports */
  }MCP23017_Handle_t;
  
  /* Exported Functions --------------------------------------------------------*/
  void              MCP23017_init      (MCP23017_Handle_t *hDev, I2C_HandleTypeDef *hi2c, uint16_t Addr);
  HAL_StatusTypeDef	MCP23017_read      (MCP23017_Handle_t *hDev, uint16_t Reg, uint8_t * pData);
  HAL_StatusTypeDef	MCP23017_write     (MCP23017_Handle_t *hDev, uint16_t Reg, uint8_t * pData);
  HAL_StatusTypeDef	MCP23017_iodir     (MCP23017_Handle_t *hDev, uint8_t Port, uint8_t Iodir);
  HAL_StatusTypeDef	MCP23017_ipol      (MCP23017_Handle_t *hDev, uint8_t Port, uint8_t Ipol);
  HAL_StatusTypeDef	MCP23017_ggpu      (MCP23017_Handle_t *hDev, uint8_t Port, uint8_t Pup);
  HAL_StatusTypeDef	MCP23017_read_gpio (MCP23017_Handle_t *hDev, uint8_t Port);
  HAL_StatusTypeDef	MCP23017_write_gpio(MCP23017_Handle_t *hDev, uint8_t Port);

#endif    /* _MCP23017_H */
