/**
  * @file           datamanagement.c
  * @details        This file provides datamanagement's functions definition.
  * @author         Shima Nosrati
  * @Date           1400.07.03
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        
  */

/* Includes ------------------------------------------------------------------*/
#include "datamanagement.h"

/* Private Functions ---------------------------------------------------------*/
/**
  * @brief   This function provides calculation of crc16 of pData with lenth of Length. 
  *          CRC-16-CCITT/XMODEM algorithms is used (Polynomial = 0x1021)
  * @param   pData specifies the pointer to data that crc16 of it should be calculate.
  * @param   Length specifies the length of data that crc16 of it should be calculate.
  * @returns The value of calculated crc16
  * @note    None
  */
static uint16_t crc16 (const uint8_t * pData, uint8_t Length)
{
  /**
    * @var   uint16_t CrcValue
    * @brief This variable holds the value of calculated crc16
    *        Initialize CrcValue to starting value (in XMODEM  init = 0x0000)
    */
  uint16_t CrcValue = 0x0000;
  
  /**
    * @var   uint16_t CalcTmp
    * @brief This variable holds temporary calculation values.
    */  
  uint8_t CalcTmp;
  
  /**
    * @var   uint16_t DataTmp 
    * @brief This variable holds the value of bytes that we want to calculated crc of them
             Initialize DataTmp to starting value
    */
  uint8_t DataTmp = 0;
  
  /*
   * Divide the message by the polynomial, a byte at a time.
   */
  while (Length--)
  {
    DataTmp = *pData;
  
    /* Bring the next byte into the CalcTmp */
    CalcTmp = (uint8_t)((CrcValue >> 8) ^ DataTmp);
    CalcTmp ^= (uint8_t)(CalcTmp >> 4);
    CrcValue = (uint16_t)( ((CrcValue << 8)) ^
                           (((uint16_t)CalcTmp) << 12) ^
                           (((uint16_t)CalcTmp) << 5) ^
                           ((uint16_t)CalcTmp) );
    pData++;
  }
  
  return CrcValue;
}

/* Exported Functions --------------------------------------------------------*/
/**
  * @brief   This function analyze the received data from Uart. 
  *          In this function termination and crc is checked and Changes the data and command based on the received data.
  * @param   pData specifies the pointer to data that should be analyzed(Recieved bytes from Uart).
  * @param   pRxCnt specifies the pointer to counter of received data from Uart.
  * @param   pDataAndCommand specifies the pointer to Data And Command structure(receive commands and the received Command Data from Uart).
  * @returns None
  * @note    None
  */
void input_packet_analyzer (uint8_t *pRxData, uint8_t *pRxCnt, DataAndCommand_t *pDataAndCommand)
{
  /**
    * @var   uint16_t Crc
    * @brief This variable holds the value of Crc
    */
  uint16_t Crc;
  
  /**
    * @var   ByteExtractor_t ByteExtract
    * @brief This variable used to joining two uint8_t bytes into one uint16_t
    */
  ByteExtractor_t ByteExtract;
  
  /**
    * @var   uint8_t RxDataTmp[8]
    * @brief This variable holds the array of correct receive data (8 bytes data that checked and has correct crc and termination)
    */
  uint8_t RxDataTmp[8];
  
  /**
    * @var   uint8_t RxDataTmp[8]
    * @brief This variable holds the counter of correct received data
    */
  uint8_t RxCounterTmp = 0;
  
  /**
    * @note Initialize internal variables
    */
  pDataAndCommand->CmdType = CmdNone;
  RxCounterTmp = 0;
  
  /**
    * @note Fill 8 correct received bytes to RxDataTmp
    */
  while (RxCounterTmp < 8)
  {
    RxDataTmp[RxCounterTmp] = *((uint8_t *)(pRxData + *pRxCnt - 8 + RxCounterTmp));
    RxCounterTmp++;
  }
  *pRxCnt = 0;
  
  /**
    * @note check out termination conditions
    */
  if ((RxDataTmp[6] == 0x0D) && (RxDataTmp[7] == 0x0A))
  {
    /**
      * @note If the first byte was 0x47 and crc was correct, change the command to CmdGet and fill CmdData with dummy data
      *       If the first byte was 0x4C and crc was correct, change the command to CmdLed and fill CmdData with received data
      */
    if (RxDataTmp[0] == 0x47)
    {
      ByteExtract.Bytes[0] = RxDataTmp[5];
      ByteExtract.Bytes[1] = RxDataTmp[4];
      
      Crc = crc16(RxDataTmp, 4);

      if (Crc == ByteExtract.HalfWord)
      {
        pDataAndCommand->CmdType = CmdGet;
        pDataAndCommand->CmdData[0] = 0x00;
        pDataAndCommand->CmdData[1] = 0x00;
        pDataAndCommand->CmdData[2] = 0x00;
      }
    }
    else if (RxDataTmp[0] == 0x4C)
    {
      ByteExtract.Bytes[0] = RxDataTmp[5];
      ByteExtract.Bytes[1] = RxDataTmp[4];
      
      Crc = crc16(RxDataTmp, 4);
      
      if (Crc == ByteExtract.HalfWord)
      {
        pDataAndCommand->CmdType = CmdLed;
        pDataAndCommand->CmdData[0] = RxDataTmp[1];
        pDataAndCommand->CmdData[1] = RxDataTmp[2];
        pDataAndCommand->CmdData[2] = RxDataTmp[3];
      }
    }
    else
    {
      /* Empty */
    }
  }
}

/**
  * @brief   This function prepares the packet to send to the controller. 
  * @param   pPacketData Specifies the pointer to buffer packet(contains ACK and STATUS data) and list of messages.
  * @param   pStatusData Specifies the pointer to list of status data (values of joystick and speed potentiometers positions and Buttons situation).
  * @returns None
  * @note    None
  */
void make_message (PacketData_t *pPacketData, StatusData_t *pStatusData)
{
  /**
    * @var   uint16_t Crc
    * @brief This variable holds the value of Crc.
    */
  uint16_t Crc;
  
  /**
    * @var   ByteExtractor_t ByteExtract
    * @brief This variable used to separate two uint8_t bytes from a uint16_t.
    */
  ByteExtractor_t ByteExtract;
  
  /**
    * @note Initialize termination data of buffer.
    */
  pPacketData->Buffer[9 ] = 0x0D;
  pPacketData->Buffer[10] = 0x0A;
  
  /**
    * @note Fill buffer based on the message type.
    */
  if (pPacketData->MsgType == MsgStatus)
  {
    pPacketData->Buffer[0] = 0x53;
    pPacketData->Buffer[1] = pStatusData->Xjoy;
    pPacketData->Buffer[2] = pStatusData->Yjoy;
    pPacketData->Buffer[3] = pStatusData->Zjoy;
    pPacketData->Buffer[4] = pStatusData->Speed;
    pPacketData->Buffer[5] = pStatusData->Keypad1;
    pPacketData->Buffer[6] = pStatusData->Keypad2;
  }
  else if(pPacketData->MsgType == MsgAck)
  {
    pPacketData->Buffer[0] = 0x41;
    pPacketData->Buffer[1] = 0x00;
    pPacketData->Buffer[2] = 0x00;
    pPacketData->Buffer[3] = 0x00;
    pPacketData->Buffer[4] = 0x00;
    pPacketData->Buffer[5] = 0x00;
    pPacketData->Buffer[6] = 0x00;
  }
  
  /**
    * @note calculate crc and put it in Buffer[7] ans Buffer[8].
    */
  Crc = crc16(pPacketData->Buffer , 7);
  ByteExtract.HalfWord = Crc;
  pPacketData->Buffer[7] = ByteExtract.Bytes[1];
  pPacketData->Buffer[8] = ByteExtract.Bytes[0];
}
