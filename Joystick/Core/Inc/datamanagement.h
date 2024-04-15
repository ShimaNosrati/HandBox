/**
  * @file           datamanagement.h
  * @details        This file provides definition of new types and function's prototypes for data manipulation.
  * @author         Shima Nosrati
  * @Date           1400.07.03
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DATAMANAGEMENT_H
  #define _DATAMANAGEMENT_H
    
  /* Includes ------------------------------------------------------------------*/ 
  #include "stm32f0xx_hal.h"
  
  /* Exported Types ------------------------------------------------------------*/
  /**
    * @union ByteExtractor_t
    * @brief Use this union to separate two uint8_t bytes from a uint16_t or joining two uint8_t bytes into one uint16_t
    */
  typedef union
  {
    uint16_t HalfWord;  /**< contain two bytes  */
    uint8_t Bytes[2];   /**< array of two bytes */ 
  }ByteExtractor_t;

  /**
    * @struct StatusData_t
    * @brief  This struct holds a list of status buffer data (values of joystick and speed potentiometers positions and Buttons situation)
    *         value of Joystick axis position for negative position is between 0-99, 
    *         for Positive position is between 101-200, when the Joystick is released is 100
    *         value of Speed potentiometer position is between 0-10
    */
  typedef struct
  {
    uint8_t Xjoy;      /**< Holds the value of Joystick X-axis position     */
    uint8_t Yjoy;      /**< Holds the value of Joystick Y-axis position     */
    uint8_t Zjoy;      /**< Holds the value of Joystick Z-axis position     */
    uint8_t Speed;     /**< Holds the value of Speed potentiometer position */
    uint8_t Keypad1;   /**< Holds the value of first 8Keys situation        */
    uint8_t Keypad2;   /**< Holds the value of second 8Keys situation       */
  }StatusData_t;

  /**
    * @enum  RecievedCommand_t
    * @brief This struct holds a list of receive commands
    */
  typedef enum
  {
    CmdNone = 0,   /**< None        */
    CmdGet = 1,    /**< Get command */
    CmdLed = 2     /**< Led command */
  }RecievedCommand_t;

  /**
    * @struct DataAndCommand_t
    * @brief  This struct Includes receive commands and the received Command Data from Uart
    */
  typedef struct
  {
    RecievedCommand_t CmdType;   /**< A list of receive commands                       */
    uint8_t CmdData[3];          /**< The received Command Data (LEDs and Buzzer data) */
  }DataAndCommand_t;

  /**
    * @enum  SendMessage_t
    * @brief This struct holds a list of messages
    */
  typedef enum
  {
    MsgNone = 0,    /**< no message to send          */
    MsgStatus = 1,  /**< Status message must be sent */
    MsgAck = 2      /**< Ack message must be sent    */
  }SendMessage_t;

  /**
    * @struct PacketData_t
    * @brief  This struct Includes Buffer(Ack ans Status) packet  and a list of messages (This packet is filled to send to the controller)
    */
  typedef struct
  {
    SendMessage_t MsgType;   /**< list of message types         */
    uint8_t Buffer[11];      /**< Buffer(Ack ans Status) packet */
  }PacketData_t;
  
  /* Exported Functions --------------------------------------------------------*/
  void input_packet_analyzer (uint8_t *pRxData, uint8_t *pRxCnt, DataAndCommand_t *pDataAndCommand);
  void make_message          (PacketData_t *pPacketData, StatusData_t *pStatusData);

#endif    /* _DATAMANAGEMENT_H */
