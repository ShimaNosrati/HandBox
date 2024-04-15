/**
  * @file           mcp2317.c
  * @details        This file provides MCP23017's functions definition.
  * @author         Shima Nosrati
  * @Date           1400.06.17
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        
  */

/* Includes ------------------------------------------------------------------*/
#include "mcp23017.h"

/* Exported Functions --------------------------------------------------------*/
/**
  * @brief   This function initializes MCP23017
  * @param   hDev Handle for MCP23017 device 
  * @param   hi2c Handle for i2c
  * @param   Addr The address of MCP23017 device
  * @returns None
  * @note    This function has nothing to do with I2C module and only specifies which I2C is used and  MCP23017 address
  */
void MCP23017_init(MCP23017_Handle_t *hDev, I2C_HandleTypeDef *hi2c, uint16_t Addr)
{
	hDev->hI2c = hi2c;
	hDev->Addr = (uint16_t)(Addr << 1);
}

/**
  * @brief   This function read one byte data from a register in MCP23017
  * @param   hDev Handle for MCP23017 device
  * @param   Reg Address of MCP23017 internal register to be read
  * @param   pData Poiter to the variable that holds the read value
  * @returns Status of i2c read operation
  * @note    Size of internal registers is one byte 
  */
HAL_StatusTypeDef	MCP23017_read(MCP23017_Handle_t *hDev, uint16_t Reg, uint8_t * pData)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c read operation
    */
  HAL_StatusTypeDef Status = HAL_ERROR;
  
	Status = HAL_I2C_Mem_Read(hDev->hI2c, hDev->Addr, Reg, 1, pData, 1, MCP23017_I2C_TIMEOUT);
  
  return Status;
}

/**
  * @brief   This function write one byte data to a register in MCP23017
  * @param   hDev Handle for MCP23017 device
  * @param   Reg Address of MCP23017 internal register to be written
  * @param   pData Poiter to the variable that holds the value to be written
  * @returns Status of i2c write operation
  * @note    Size of internal registers is one byte 
  */
HAL_StatusTypeDef	MCP23017_write(MCP23017_Handle_t *hDev, uint16_t Reg, uint8_t * pData)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c write operation
    */
  HAL_StatusTypeDef Status = HAL_ERROR;
  
	Status = HAL_I2C_Mem_Write(hDev->hI2c, hDev->Addr, Reg, 1, pData, 1, MCP23017_I2C_TIMEOUT);
  
  return Status;
}

/**
  * @brief   This function sets the direction of I/Os
  * @param   hDev Handle for MCP23017 device
  * @param   Port Name of MCP23017 port
  * @param   Iodir Specifies each pin direction
  * @returns Status of i2c write operation
  * @note    refer to 3.5.1 section of datasheet
  */
HAL_StatusTypeDef	MCP23017_iodir(MCP23017_Handle_t *hDev, uint8_t Port, uint8_t Iodir)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c write operation
    */
  HAL_StatusTypeDef Status = HAL_ERROR;
  
  /**
    * @var   uint8_t Data
    * @brief This variable holds the value of MCP23017 internal register for I/O direction
    */
	uint8_t Data = Iodir;
  
	Status = MCP23017_write(hDev, MCP23017_REG_IODIRA | Port, &Data);
  
  return Status;
}

/**
  * @brief   This function sets the input polarity of I/O
  * @param   hDev Handle for MCP23017 device
  * @param   Port Name of MCP23017 port
  * @param   Ipol Specifies each pin input polarity
  * @returns Status of i2c write operation
  * @note    refer to 3.5.2 section of datasheet
  */
HAL_StatusTypeDef	MCP23017_ipol(MCP23017_Handle_t *hDev, uint8_t Port, uint8_t Ipol)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c write operation
    */
	HAL_StatusTypeDef Status = HAL_ERROR;
  
  /**
    * @var   uint8_t Data
    * @brief This variable holds the value of MCP23017 internal register for I/O input polarity
    */
	uint8_t Data = Ipol;
  
	Status = MCP23017_write(hDev, MCP23017_REG_IPOLA | Port, &Data);
  
  return Status;
}

/**
  * @brief   This function sets the pullup of Inputs
  * @param   hDev Handle for MCP23017 device
  * @param   Port Name of MCP23017 port
  * @param   Pup Specifies each pin (when configured as an input) pullup 
  * @returns Status of i2c write operation
  * @note    refer to 3.5.7 section of datasheet
  */
HAL_StatusTypeDef	MCP23017_ggpu(MCP23017_Handle_t *hDev, uint8_t Port, uint8_t Pup)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c write operation
    */
	HAL_StatusTypeDef Status = HAL_ERROR;
  
  /**
    * @var   uint8_t Data
    * @brief This variable holds the value of MCP23017 internal register for I/O pullup
    */
	uint8_t Data = Pup;
  
	Status = MCP23017_write(hDev, MCP23017_REG_GPPUA | Port, &Data);
  
  return Status;
}

/**
  * @brief   This function read data from a port of MCP23017
  * @param   hDev Handle for MCP23017 device
  * @param   Port Name of MCP23017 port to be read
  * @returns Status of i2c read operation
  * @note    None
  */
HAL_StatusTypeDef	MCP23017_read_gpio(MCP23017_Handle_t *hDev, uint8_t Port)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c read operation
    */
  HAL_StatusTypeDef Status = HAL_ERROR;
  
  /**
    * @var   uint8_t Data
    * @brief This variable holds the value to be read
    */
  uint8_t Data;
  
	Status = MCP23017_read(hDev, (MCP23017_REG_GPIOA | Port), &Data);
	if (Status == HAL_OK)
  {
		hDev->Gpio[Port] = Data;
  }
  
	return Status;
}

/**
  * @brief   This function write one byte data to a register in MCP23017
  * @param   hDev Handle for MCP23017 device
  * @param   Port Name of MCP23017 port to be written
  * @returns Status of i2c write operation
  * @note    None
  */
HAL_StatusTypeDef	MCP23017_write_gpio(MCP23017_Handle_t *hDev, uint8_t Port)
{
  /**
    * @var   HAL_StatusTypeDef Status
    * @brief This variable holds the status of i2c write operation
    */
  HAL_StatusTypeDef Status = HAL_ERROR;
  
  /**
    * @var   uint8_t Data
    * @brief This variable holds the value to be written
    */
  uint8_t Data = hDev->Gpio[Port];
  
	Status = MCP23017_write(hDev, (MCP23017_REG_GPIOA | Port), &Data);
  
  return Status;
}
