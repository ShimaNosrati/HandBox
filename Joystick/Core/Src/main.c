/* USER CODE BEGIN Header */
/**
  * @file           main.c
  * @details        "At first Should press(F1+F2)keys for 3seconds to initialize reference values of joystick(when joystick is released)"
  *                 In this code we recieve 2 Uart packages (Get and Led)
  *                 and according to them, if we recieved Get package, we 
  *                 send the status of potentiometers(ADC) and keys(I2C) 
  *                 with Status packet to controller.if we recieved Led
  *                 package, we set status of LEDs(I2C) and buzzer according
  *                 to recieved data and then send Ack packet to controller.
  *                 At first or when we didn't recieve any data from controller for 10 seconds,
  *                 the (JOYSTICK,EMERGENCY,DCC) LEDs blinking.
  *                 At first Should press(F1+F2)keys for 3seconds to initialize reference values of joystick(when joystick is released)
  *                 When user wants to reset joystick reference values should press(F1+F2)keys for 3seconds(KEYPAD_CHECK_COUNTER_MAX * 100ms)
  * @author         Shima Nosrati
  * @Date           1400.06.17
  * @copyright      Mehr (Meghyas Hamisheh Radyab)
  * @warning        ---
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"       /**< round function */
#include "mcp23017.h"
#include "datamanagement.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  * @struct AdcVlaues_t
  * @brief  This struct holds a list of read values from ADC. These variables are defined globally and 
  *         change only in HAL_ADC_ConvCpltCallback function, they should not change elsewhere in the program.
  */
typedef struct
{
  uint16_t Xpos;   /**< This variable holds the read adc value of Joystick X-axis (0-4095 for 12 bit resolution)     */
  uint16_t Ypos;   /**< This variable holds the read adc value of Joystick Y-axis (0-4095 for 12 bit resolution)     */
  uint16_t Zpos;   /**< This variable holds the read adc value of Joystick Z-axis (0-4095 for 12 bit resolution)     */
  uint16_t Spos;   /**< This variable holds the read adc value of Speed potentiometer (0-4095 for 12 bit resolution) */
}AdcVlaues_t;

/**
  * @struct JoystickReferences_t
  * @brief  This struct holds a list of reference values of joystick when the joystick is released
  */
typedef struct
{
  uint16_t XjoyRef;   /**< This variable holds the reference value of Joystick X-axis when the joystick is released */
  uint16_t YjoyRef;   /**< This variable holds the reference value of Joystick Y-axis when the joystick is released */
  uint16_t ZjoyRef;   /**< This variable holds the reference value of Joystick Z-axis when the joystick is released */
}JoystickReferences_t;

/**
  * @struct Flags_t
  * @brief  This struct holds a list of flags that indicate the received data from serial and read data from adc is ready or not
  */
typedef struct
{
  uint8_t RxPacketReady : 1;      /**< Indicate the received data from Uart is ready or not              */
  uint8_t AdcDataReady : 1;       /**< Indicate the read data from adc is ready or not                   */
  uint8_t TransactionExist : 1;   /**< Indicate there is data transaction between Handbox and Controller */
  uint8_t JoyRefSet : 1;          /**< Indicate the JoyStick reference values are set or not             */
  uint8_t Dummy : 4;              /**< Not used                                                          */
}Flags_t;

/**
  * @enum BlinkStatus_t
  * @brief Indicate JOYSTICK,EMERGENCY,DCC LEDs are OFF or ON
  */
typedef enum
{
  BlinkOff = 0,  /**< JOYSTICK,EMERGENCY,DCC LEDs are OFF */
  BlinkOn = 1    /**< JOYSTICK,EMERGENCY,DCC LEDs are ON  */
}BlinkStatus_t;

  
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
  * @def   RX_BUFFER_MAX_SIZE
  * @brief Specify the maximum value of RxBuffer size
  */
#define RX_BUFFER_MAX_SIZE    ((uint8_t)50)

/**
  * @def   TX_BUFFER_MAX_SIZE
  * @brief Specify the maximum value of TxBuffer size
  */
#define TX_BUFFER_MAX_SIZE    ((uint8_t)11)

/**
  * @def   ADC_SAMPLE_NUMBER
  * @brief Specify the number of samples from each adc channel (must be even and power of 2)
  */
#define ADC_SAMPLE_NUMBER     ((uint8_t)16)

/**
  * @def   ADC_SAMPLE_SHIFT_RIGHT
  * @brief Specify the number of shifting right the value of adc sampling to calculate value of average of adc sampling data
  *        (ADC_SAMPLE_SHIFT_RIGHT = log(ADC_SAMPLE_NUMBER) in based 2)
  */
#define ADC_SAMPLE_SHIFT_RIGHT     ((uint8_t)4)

/**
  * @def   ADC_BUFFER_MAX_SIZE
  * @brief Specify the maximum value of AdcBuffer size
  */
#define ADC_BUFFER_MAX_SIZE   ((uint8_t)(ADC_SAMPLE_NUMBER * ((uint8_t)4)))

/**
  * @def   ADC_VALUE_UPPER_LIMIT
  * @brief Specify the value of maximum limit of read adc data (4020 = 4095 - 75)
  */
#define ADC_VALUE_UPPER_LIMIT   ((uint16_t)4020)

/**
  * @def   ADC_VALUE_LOWER_LIMIT
  * @brief Specify the value of minimum limit of read adc data (75 = 0 + 75)
  */
#define ADC_VALUE_LOWER_LIMIT   ((uint16_t)75)

/**
  * @def   ADC_REF_OFFSET
  * @brief Specify the value of Offset (As this value increases, the accuracy of adc value decreases)
  *        We used this offset to eliminate potentiometers position instability 
  */
#define ADC_REF_OFFSET    ((uint16_t)20)

/**
  * @def   TRANSACTION_TIME_COUNTER_MAX
  * @brief Specify the maximum value of counter Which is used to calculate Transaction time
  *        Transaction time = counter * 500ms (500ms is when the timer interrupt occurs)
  */
#define TRANSACTION_TIME_COUNTER_MAX    ((uint16_t)20)

/**
  * @def   PAGE63_ADRS
  * @brief Specify the address of Page 63 of microcontroller flash.
  */
#define PAGE63_ADRS    ((uint32_t)0x0800FC00)

/**
  * @def   JOYREF_CHECK_ADRS
  * @brief Specify the address where JOYREF_CHECK_VALUE is stored, which indicates the reference values are set or not.
  *        When the keys (F1, F2) are pressed, we store a specific value of JOYREF_CHECK_VALUE in JOYREF_CHECK_ADRS address, 
  *        and we also store the reference values in the specified addresses( XJOYREF_ADRS, YJOYREF_ADRS, ZJOYREF_ADRS), 
  *        and by examining the specified value stored in address JOYREF_CHECK_ADRS, we find that the reference values are set Or not.
  *        (Refer to Table-4 of stm32f030 user manual)
  */
#define JOYREF_CHECK_ADRS    (PAGE63_ADRS + ((uint32_t)0))

/**
  * @def   XJOYREF_ADRS
  * @brief Specify the microcontroller flash address where value of reference value of joystick position in X-axis, will be stored.
  *        (Refer to Table-4 of stm32f030 user manual)
  */
#define XJOYREF_ADRS    (PAGE63_ADRS + ((uint32_t)2))

/**
  * @def   YJOYREF_ADRS
  * @brief Specify the microcontroller flash address where value of reference value of joystick position in Y-axis, will be stored.
  *        (Refer to Table-4 of stm32f030 user manual)
  */
#define YJOYREF_ADRS    (PAGE63_ADRS + ((uint32_t)4))

/**
  * @def   ZJOYREF_ADRS
  * @brief Specify the microcontroller flash address where value of reference value of joystick position in Z-axis, will be stored.
  *        (Refer to Table-4 of stm32f030 user manual)
  */
#define ZJOYREF_ADRS    (PAGE63_ADRS + ((uint32_t)6))

/**
  * @def   JOYREF_CHECK_VALUE
  * @brief Specify the value that stored in JOYREF_CHECK_ADRS address and indicates the reference values are set or not.
  */
#define JOYREF_CHECK_VALUE    ((uint16_t)0xAAAA)

/**
  * @def   KEYPAD_CHECK_COUNTER_MAX
  * @brief Specify the counter, which specifies the time that (F1, F2) keys must be pressed (KEYPAD_CHECK_COUNTER_MAX * 100ms).
  *        In KEYPAD_CHECK_COUNTER_MAX=30, user must press the keys for 3 seconds to save the joystick reference values.
  */
#define KEYPAD_CHECK_COUNTER_MAX ((uint16_t)30)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/**
  * @var   const uint16_t PositiveMax 
  * @brief This variable holds the maximum value of Joystick axis position for positive position
  */
const uint16_t PositiveMax = 200;

/**
  * @var   const uint16_t PositiveMin
  * @brief This variable holds the minimum value of Joystick axis position for positive position
  */
const uint16_t PositiveMin = 101;

/**
  * @var   const uint16_t NegativeMax
  * @brief This variable holds the maximum value of Joystick axis position for negative position
  */
const uint16_t NegativeMax = 99;

/**
  * @var   const uint16_t NegativeMin
  * @brief This variable holds the minimum value of Joystick axis position for negative position
  */
const uint16_t NegativeMin = 0;

/**
  * @var   const uint16_t SpeedMax
  * @brief This variable holds the maximum value of Speed potentiometer position
  */
const uint16_t SpeedMax = 10;

/**
  * @var   const uint16_t SpeedMin
  * @brief This variable holds the minimum value of Speed potentiometer position
  */
const uint16_t SpeedMin = 0;

/**
  * @var   uint8_t ReceivedData
  * @brief This variable holds the value of 1byte recieved data from Uart
  */
uint8_t ReceivedData;

/**
  * @var   uint8_t RxBuffer[RX_BUFFER_MAX_SIZE]
  * @brief Recieved bytes from Uart are stored in this buffer. This variable is defined globally and 
  *        change only in HAL_UART_RxCpltCallback function, it should not change elsewhere in the program.
  */
uint8_t RxBuffer[RX_BUFFER_MAX_SIZE];

/**
  * @var   uint8_t RxCounter
  * @brief This variable holds the value of the counter of received data
  *        Initialize RxCounter to starting value
  */
uint8_t RxCounter = 0;

/**
  * @var   uint16_t AdcBuffer[16]
  * @brief The data read from adc is stored in this buffer
  */
uint16_t AdcBuffer[ADC_BUFFER_MAX_SIZE];

/**
  * @var   MCP23017_Handle_t hmcp1
  * @brief This variable contains value and parameters of First MCP23017 IC
  */
MCP23017_Handle_t hmcp1;

/**
  * @var   MCP23017_Handle_t hmcp2
  * @brief This variable contains value and parameters of second MCP23017 IC
  */
MCP23017_Handle_t hmcp2;

/**
  * @var   AdcVlaues_t AdcReadValues
  * @brief This variable holds the read adc value of Joystick and Speed potentiometers. These variables are defined globally and 
  *        change only in HAL_ADC_ConvCpltCallback function, they should not change elsewhere in the program.
  */
AdcVlaues_t AdcReadValues;

/**
  * @var   Flags_t Flag
  * @brief This variable holds a list of flags that indicate the received data from Uart and read data from adc is ready or not
  */
Flags_t Flag;

/**
  * @var   uint16_t TransactionTimeCounter
  * @brief This variable holds the counter Which is used to calculate Transaction time
  */
uint16_t TransactionTimeCounter = 0;

/**
  * @var   BlinkStatus_t BlinkStatus
  * @brief This variable holds the status of JOYSTICK,EMERGENCY,DCC LEDs.
  */
BlinkStatus_t BlinkStatus = BlinkOff;

uint16_t ReadFlash;

uint16_t KeypadCheckCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void get_status_routine(StatusData_t *pStatusData, JoystickReferences_t *pJoyRef, AdcVlaues_t *pAdcVals);
void led_status_routine(DataAndCommand_t *pDataAndCmd);
void buzzer (uint16_t BeepTime);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /**
    * @var   DataAndCommand_t DataAndCommand
    * @brief This variable holds receive commands and the received Command Data from Uart
    */
  DataAndCommand_t DataAndCommand;

  /**
    * @var   StatusData_t StatusData
    * @brief This variable holds a list of status buffer data (values of joystick and speed potentiometer positions and Buttons situation)
    */
  StatusData_t StatusData;
  
  /**
    * @var   PacketData_t PacketData
    * @brief This variable holds Buffer(Ack ans Status) packet  and a list of messages (This packet is filled to send to the controller)
    */
  PacketData_t PacketData;
  
  /**
    * @var   JoystickReferences_t JoyRef
    * @brief This variable holds a list of reference values of joystick when the joystick is released
    */
  JoystickReferences_t JoyRef;
  
  /**
    * @var   FLASH_EraseInitTypeDef EraseInitStruct
    * @brief FLASH Erase structure definition
    */
  FLASH_EraseInitTypeDef EraseInitStruct;
  
  /**
    * @var   uint32_t PageError
    * @brief This variable contains the configuration information on faulty page in case of error
    *        (0xFFFFFFFF means that all the pages have been correctly erased)
    *        Initialize value to zero.
    */
  uint32_t PageError = 0;
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  
  /**
    * @note Initialize MCP23017 ICs. define their address and their direction(input-output) and initialize all leds are OFF.
    */
  MCP23017_init(&hmcp1, &hi2c2, MCP23017_ADDRESS_1);
  MCP23017_init(&hmcp2, &hi2c2, MCP23017_ADDRESS_8);
  MCP23017_iodir(&hmcp1, MCP23017_PORTA, MCP23017_IODIR_ALL_OUTPUT);
  MCP23017_iodir(&hmcp1, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  MCP23017_iodir(&hmcp2, MCP23017_PORTA, MCP23017_IODIR_ALL_OUTPUT);
  MCP23017_iodir(&hmcp2, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  
  /* LEDs OFF */
  hmcp1.Gpio[MCP23017_PORTA] = 0xFF;
  MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
  hmcp2.Gpio[MCP23017_PORTA] = 0xFF;
  MCP23017_write_gpio(&hmcp2, MCP23017_PORTA);
  
  /**
    * @note Initialize variable to starting value. (no flag, no command, no message)
    */
  Flag.JoyRefSet = 0;
  Flag.RxPacketReady = 0;
  Flag.AdcDataReady = 0;
  Flag.TransactionExist = 0;
  BlinkStatus = BlinkOff;
  TransactionTimeCounter = 0;
  DataAndCommand.CmdType = CmdNone;
  DataAndCommand.CmdData[0] = 0x00;
  DataAndCommand.CmdData[1] = 0x00;
  DataAndCommand.CmdData[2] = 0x00;
  PacketData.MsgType = MsgNone;
  KeypadCheckCounter = 0;
  
  HAL_Delay(500);
  /**
    * @note Read Flash to check if reference values were previously set.
    *       When the keys (F1, F2) are pressed, we store a specific value of JOYREF_CHECK_VALUE in JOYREF_CHECK_ADRS address, 
    *       and we also store the reference values in the specified addresses( XJOYREF_ADRS, YJOYREF_ADRS, ZJOYREF_ADRS), 
    *       and by examining the specified value stored in address JOYREF_CHECK_ADRS, we find that the reference values are set Or not.
    *       and if reference values were previously set we use that values(that saved in flash memory) for reference values.
    */
  ReadFlash = *(uint16_t *)JOYREF_CHECK_ADRS;
  if (ReadFlash == JOYREF_CHECK_VALUE)
  {
    JoyRef.XjoyRef = *(uint16_t *)XJOYREF_ADRS;
    JoyRef.YjoyRef = *(uint16_t *)YJOYREF_ADRS;
    JoyRef.ZjoyRef = *(uint16_t *)ZJOYREF_ADRS;
    Flag.JoyRefSet = 1;
  }
  else
  {
    JoyRef.XjoyRef = 0;
    JoyRef.YjoyRef = 0;
    JoyRef.ZjoyRef = 0;
    Flag.JoyRefSet = 0;
  
  }
  /**
    * @note Initial beep.
    */
  buzzer(50);
  
  /* Start and enable interupt for  receive data(one byte) from uart */
  HAL_UART_Receive_IT(&huart1, &ReceivedData, 1);
  /* Start and enable interupt for 500ms tick timer */
  HAL_TIM_Base_Start_IT(&htim3);
  /* Start and enable interupt for 100ms tick timer to check keypad */
  HAL_TIM_Base_Start_IT(&htim6);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for (;;)
  {
   
    /**
      * @note When received data from uart is ready reset the RxPacketReady and analyze the received data from Uart. 
      */
    if ((Flag.RxPacketReady == 1) && (Flag.JoyRefSet == 1))
    {
      Flag.RxPacketReady = 0;
      
      input_packet_analyzer(RxBuffer, &RxCounter, &DataAndCommand);
    }
    
    /**
      * @note When received command is CmdGet routine of Get command is performed The message type changes to 
      *       MsgStatus and reset command and command data values. When received command is CmdLed routine of Led
      *       command is performed The message type changes to MsgAck and reset command and command data values.
      */
    if ((DataAndCommand.CmdType == CmdGet) && (Flag.JoyRefSet == 1))
    {
      get_status_routine(&StatusData, &JoyRef, &AdcReadValues);
      /*  Change The message type to MsgStatus */
      PacketData.MsgType = MsgStatus;
      
      /*  Reset Command  */
      DataAndCommand.CmdType = CmdNone;
      
      /*  Reset command data values  */
      DataAndCommand.CmdData[0] = 0x00;
      DataAndCommand.CmdData[1] = 0x00;
      DataAndCommand.CmdData[2] = 0x00;
    }
    else if ((DataAndCommand.CmdType == CmdLed) && (Flag.JoyRefSet == 1))
    {
      led_status_routine(&DataAndCommand);
      
      /*  Change The message type to MsgAck  */
      PacketData.MsgType = MsgAck;
      
      /*  Reset Command  */
      DataAndCommand.CmdType = CmdNone;
      
      /*  Reset command data values  */
      DataAndCommand.CmdData[0] = 0x00;
      DataAndCommand.CmdData[1] = 0x00;
      DataAndCommand.CmdData[2] = 0x00;
    }
    else
    {
      /* Empty */
    }
    
    /**
      * @note When the message isn't MsgNone, make message (prepare message to send to controller) and using Uart send it to controller.
      */
    if ((PacketData.MsgType != MsgNone) && (Flag.JoyRefSet == 1))
    {
      make_message(&PacketData, &StatusData);
      
      /*  Send PacketData.Buffer to controller using uart */
      HAL_UART_Transmit(&huart1, PacketData.Buffer, TX_BUFFER_MAX_SIZE, 100); 
      
      /*  Reset The message type to MsgNone  */
      PacketData.MsgType = MsgNone;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    
    /**
      * @note When (F1, F2) keys are pressed for (KEYPAD_CHECK_COUNTER_MAX * 100ms) secons, stop all timers and turn on all LEDs
      *       and read Adc and save the joystick reference values on the microcontroller flash memory. Then Turn off all LEDs and
      *       start timer3 and timer6 to continue the program.
      *       Stop all actions to initialize Joystick reference values then start actions for program routine.
      */
    if (KeypadCheckCounter == KEYPAD_CHECK_COUNTER_MAX)
    {
      /**
        * @note stop timer3 and timer6
        */
      HAL_TIM_Base_Stop(&htim3);
      HAL_TIM_Base_Stop(&htim6);
      
      /**
        * @note Turn on all LEDs
        */
      hmcp1.Gpio[MCP23017_PORTA] = 0x00;
      MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
      hmcp2.Gpio[MCP23017_PORTA] = 0x00;
      MCP23017_write_gpio(&hmcp2, MCP23017_PORTA);
      
      /**
        * @note read Adc
        */
      Flag.AdcDataReady = 0;
      HAL_ADC_Start_DMA(&hadc,(uint32_t*) AdcBuffer, ADC_BUFFER_MAX_SIZE);
      while (Flag.AdcDataReady == 0);
      
      /**
        * @note save the joystick reference values on the microcontroller flash memory(page63, Refer to Table-4 of stm32f030 user manual).
        */
      HAL_FLASH_Unlock();
      
      EraseInitStruct.NbPages = 1;
      EraseInitStruct.PageAddress = PAGE63_ADRS;
      EraseInitStruct.TypeErase = TYPEERASE_PAGES;
      HAL_FLASHEx_Erase(&EraseInitStruct , &PageError);
      
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, JOYREF_CHECK_ADRS, JOYREF_CHECK_VALUE);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, XJOYREF_ADRS, AdcReadValues.Xpos);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, YJOYREF_ADRS, AdcReadValues.Ypos);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ZJOYREF_ADRS, AdcReadValues.Zpos);
      
      HAL_FLASH_Lock();
      
      JoyRef.XjoyRef = *(uint16_t *)XJOYREF_ADRS;
      JoyRef.YjoyRef = *(uint16_t *)YJOYREF_ADRS;
      JoyRef.ZjoyRef = *(uint16_t *)ZJOYREF_ADRS;
      Flag.JoyRefSet = 1;
      
      HAL_Delay(5000);
      
      /**
        * @note Turn off all LEDs
        */
      hmcp1.Gpio[MCP23017_PORTA] = 0xFF;
      MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
      hmcp2.Gpio[MCP23017_PORTA] = 0xFF;
      MCP23017_write_gpio(&hmcp2, MCP23017_PORTA);
      
      /**
        * @note start timer3 and timer6.
        */
      Flag.TransactionExist = 0;
      BlinkStatus = BlinkOff;
      TransactionTimeCounter = 0;
      HAL_TIM_Base_Start_IT(&htim3);
      
      KeypadCheckCounter = 0;
      HAL_TIM_Base_Start_IT(&htim6);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief   Starts the TIM Base generation in interrupt mode. This interruption occurs for timer3 and timer6.
  *          Timer3 = 500ms                    Timer6 = 600ms
  *          Timer3 = (PSC + 1) * (ARR + 1) / Ft = (4800) * (5000) / 48MHz = 500ms
  * @param   htim TIM Base handle   
  * @returns None
  * @note    None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    /**
      * @note When there is no transaction JOYSTICK,EMERGENCY,DCC LEDs blink. when handbox receives data from controller,
      *       it sets Flag of TransactionExist and reset TransactionTimeCounter and 
      *       When there is no transaction between HandBox and Controller for (TRANSACTION_TIME_COUNTER_MAX * 500ms) seconds  
      *       and TransactionTimeCounter reaches to its predefined value, reset the TransactionTimeCounter and TransactionExist
      *       flag(to make  JOYSTICK,EMERGENCY,DCC LEDs blink) and turn off all LEDs.
      */
    if (Flag.TransactionExist == 0)
    {
      if (BlinkStatus == BlinkOff)
      {
        /* Turn on JOYSTICK,EMERGENCY,DCC LEDs  */
        hmcp1.Gpio[MCP23017_PORTA] = 0xC7;
        MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
        BlinkStatus = BlinkOn;
      }
      else
      {
        /* Turn off LEDs of first MCP23017      */
        hmcp1.Gpio[MCP23017_PORTA] = 0xFF;
        MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
        BlinkStatus = BlinkOff;
      }
    }
    else
    {
      TransactionTimeCounter++;

      if (TransactionTimeCounter == TRANSACTION_TIME_COUNTER_MAX)
      {
        Flag.TransactionExist = 0;
        TransactionTimeCounter = 0;
        
        hmcp1.Gpio[MCP23017_PORTA] = 0xFF;
        MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
        hmcp2.Gpio[MCP23017_PORTA] = 0xFF;
        MCP23017_write_gpio(&hmcp2, MCP23017_PORTA);
        BlinkStatus = BlinkOff;
      }
    }
  }
  else if (htim == &htim6)
  {
     /**
      * @note Sometimes the data sent by the controller was not received by the Handbox in the program.
      *       with this part of code we can solve this problem.
      *       Every 100ms check if Rx process is not already ongoing ,enable interupt for receive data.
      */
    if (huart1.RxState == HAL_UART_STATE_READY)
    {
      HAL_UART_Receive_IT(&huart1, &ReceivedData, 1); 
    }
    
    /**
      * @note Every 100ms check F1, F2 keys and if F1, F2 are pressed increase KeypadCheckCounter
      *       and if F1, F2 are not pressed reset KeypadCheckCounter.
      */
    MCP23017_read_gpio(&hmcp1, MCP23017_PORTB);
    if (hmcp1.Gpio[MCP23017_PORTB] == 0x7E)
    {
      KeypadCheckCounter++;
    }
    else
    {
      KeypadCheckCounter = 0;
    }
  }
}

/**
  * @brief   Rx Transfer completed callback. In this function received data from Uart save in a buffer and check termination
  *          if it was correct and Flag.TransactionExist == 0, turn off all LEDs and set RxPacketReady flag And it is announced
  *          that the data is ready and set TransactionExist flag and reset TransactionTimeCounter.
  * @param   huart UART Handle  
  * @returns None
  * @note    None
  */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  /**
   * @note Fill RxBuffer with received data.
   */ 
  RxBuffer[RxCounter] = ReceivedData;
  
  /**
   * @note check termination and if termination was ok and Flag.TransactionExist == 0, turn off all LEDs set RxPacketReady flag and
   *       set TransactionExist flag and reset TransactionTimeCounter .
   */
  if ((RxBuffer[RxCounter] == 0x0A) && (RxCounter != 0))
  {
    if (RxBuffer[RxCounter - 1] == 0x0D)
    {
      /**
        * @note When there is no transaction between HandBox and Controller (JOYSTICK, EMERGENCY, DCC) LEDs are blinking and when HandBox recieved
        *       data from Controller Turn off all LEDs.
        */
      if (Flag.TransactionExist == 0) 
      {
        hmcp1.Gpio[MCP23017_PORTA] = 0xFF;
        MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
        hmcp2.Gpio[MCP23017_PORTA] = 0xFF;
        MCP23017_write_gpio(&hmcp2, MCP23017_PORTA);
        BlinkStatus = BlinkOff;
      }
      Flag.RxPacketReady = 1;
      Flag.TransactionExist = 1;
      TransactionTimeCounter = 0;
    }
  }
  /**
   * @note increase RxCounter for save next receive byte in RxBuffer.
   */
  RxCounter++;
  /**
   * @note Use this condition to prevent from RxBuffer overflow.
   */
  if (RxCounter > RX_BUFFER_MAX_SIZE)
  { 
    RxCounter = 0;
  }
  
  /**
    * @note Enable interupt for  receive the next byte.
    */
  HAL_UART_Receive_IT(&huart1, &ReceivedData, 1);
}

/**
  * @brief   Conversion complete callback in non blocking mode. When ADC_BUFFER_MAX_SIZE bytes of adc are read,
  *          it enters this function and ADC_SAMPLE_NUMBER samples are taken from each channel and their average 
  *          is taken and stored in AdcReadValues variable and set adc data ready flag And it is announced that the ADC data is ready.
  *          ADC_BUFFER_MAX_SIZE = ADC_SAMPLE_NUMBER * 4 (because we use 4 channel of adc)
  *          AdcReadValues variable is defined globally and change only in HAL_ADC_ConvCpltCallback function, it should not change elsewhere in the program.
  * @param   hadc ADC Handle 
  * @returns None
  * @note    None
  */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
  /**
    * @var   uint16_t Cnt
    * @brief This variable holds the value of counter of ADC_SAMPLE_NUMBER.
    */
  uint16_t Cnt = 0;
  
  /**
    * @var   uint16_t Channel
    * @brief This variable holds the value of adc channel number.
    */
  uint16_t Channel = 0;
  
  /**
    * @var    uint16_t Index
    * @brief This variable holds the value of index of AdcBuffer.
    */
  uint16_t Index = 0;
  
  /**
    * @var   uint16_t ReadData
    * @brief This variable holds the value of read adc data of each channel of adc.
    */
  uint16_t ReadData = 0;
  
  /**
    * @var   uint32_t Sum
    * @brief This variable holds the value of sum of read adc data of each channel of adc.
    */
  uint32_t Sum = 0;
  
  /**
    * @note Stop adc because adc is in continues mode and if we do not stop it, the program will remain in this function.
    */
  HAL_ADC_Stop_DMA(hadc);
     
  /**
    * @note read adc data of first channel of adc (channel = 0) and calculate average of them and store in AdcReadValues.Xpos.
    */
  Sum = 0;
  Channel = 0;
  for (Cnt = 0; Cnt < ADC_SAMPLE_NUMBER; Cnt++)
  {
    Index = (uint16_t)((Cnt * 4) + Channel);
    
    if (AdcBuffer[Index] > ADC_VALUE_UPPER_LIMIT)
    {
      ReadData = ADC_VALUE_UPPER_LIMIT;
    }
    else if (AdcBuffer[Index] < ADC_VALUE_LOWER_LIMIT)
    {
      ReadData = ADC_VALUE_LOWER_LIMIT;
    }
    else
    {
      ReadData = AdcBuffer[Index];
    }
    
    Sum += ReadData;
  }
  
  /**
    * @note calculate the average of adc data. Each time the sum shift to the right, sum is divided by 2.
    *       For example if ADC_SAMPLE_SHIFT_RIGHT = 4 -> (Sum >> ADC_SAMPLE_SHIFT_RIGHT) = sum/(2^4).
    */
  AdcReadValues.Xpos = (uint16_t)(Sum >> ADC_SAMPLE_SHIFT_RIGHT);
  
  /**
    * @note read adc data of second channel of adc (channel = 1) and calculate average of them and store in AdcReadValues.Ypos.
    */
  Sum = 0;
  Channel = 1;
  for (Cnt = 0; Cnt < ADC_SAMPLE_NUMBER; Cnt++)
  {
    Index = (uint16_t)((Cnt * 4) + Channel);
    
    if (AdcBuffer[Index] > ADC_VALUE_UPPER_LIMIT)
    {
      ReadData = ADC_VALUE_UPPER_LIMIT;
    }
    else if (AdcBuffer[Index] < ADC_VALUE_LOWER_LIMIT)
    {
      ReadData = ADC_VALUE_LOWER_LIMIT;
    }
    else
    {
      ReadData = AdcBuffer[Index];
    }
    
    Sum += ReadData;
  }
  
  /**
    * @note calculate the average of adc data. Each time the sum shift to the right, sum is divided by 2.
    *       For example if ADC_SAMPLE_SHIFT_RIGHT = 4 -> (Sum >> ADC_SAMPLE_SHIFT_RIGHT) = sum/(2^4).
    */
  AdcReadValues.Ypos = (uint16_t)(Sum >> ADC_SAMPLE_SHIFT_RIGHT);
    
  /**
    * @note read adc data of third channel of adc (channel = 0) and calculate average of them and store in AdcReadValues.Zpos.
    */
  Sum = 0;
  Channel = 2;
  for (Cnt = 0; Cnt < ADC_SAMPLE_NUMBER; Cnt++)
  {
    Index = (uint16_t)((Cnt * 4) + Channel);
    
    if (AdcBuffer[Index] > ADC_VALUE_UPPER_LIMIT)
    {
      ReadData = ADC_VALUE_UPPER_LIMIT;
    }
    else if (AdcBuffer[Index] < ADC_VALUE_LOWER_LIMIT)
    {
      ReadData = ADC_VALUE_LOWER_LIMIT;
    }
    else
    {
      ReadData = AdcBuffer[Index];
    }
    
    Sum += ReadData;
  }
  
  /**
    * @note calculate the average of adc data. Each time the sum shift to the right, sum is divided by 2.
    *       For example if ADC_SAMPLE_SHIFT_RIGHT = 4 -> (Sum >> ADC_SAMPLE_SHIFT_RIGHT) = sum/(2^4).
    */
  AdcReadValues.Zpos = (uint16_t)(Sum >> ADC_SAMPLE_SHIFT_RIGHT);
  
  /**
    * @note read adc data of fourth channel of adc (channel = 3) and calculate average of them and store in AdcReadValues.Spos.
    */
  Sum = 0;
  Channel = 3;
  for (Cnt = 0; Cnt < ADC_SAMPLE_NUMBER; Cnt++)
  {
    Index = (uint16_t)((Cnt * 4) + Channel);
    
    if (AdcBuffer[Index] > ADC_VALUE_UPPER_LIMIT)
    {
      ReadData = ADC_VALUE_UPPER_LIMIT;
    }
    else if (AdcBuffer[Index] < ADC_VALUE_LOWER_LIMIT)
    {
      ReadData = ADC_VALUE_LOWER_LIMIT;
    }
    else
    {
      ReadData = AdcBuffer[Index];
    }
    
    Sum += ReadData;
  }
  
  /**
    * @note calculate the average of adc data. Each time the sum shift to the right, sum is divided by 2.
    *       For example if ADC_SAMPLE_SHIFT_RIGHT = 4 -> (Sum >> ADC_SAMPLE_SHIFT_RIGHT) = sum/(2^4).
    */  
  AdcReadValues.Spos = (uint16_t)(Sum >> ADC_SAMPLE_SHIFT_RIGHT);
 
  /**
    * @note set adc data ready flag 
    */
  Flag.AdcDataReady = 1;
}

/**
  * @brief   In this function routine of Get command is performed. 
  *          This function reads the values of the Buttons situation and potentiometers and, according to the calculations,
  *          shows position Joystick potentiometers between 0-200 and Speed potentiometers between 0-10 and stores all of these values in the pStatusData.
  * @param   pJoyRef specifies the pointer to reference values of joystick when the joystick is released.
  * @param   pAdcVals specifies the pointer to  read values from ADC.
  * @param   pStatusData specifies the pointer to list of status data (values of joystick and speed potentiometers positions and Buttons situation)
  * @returns None
  * @note    None
  */
void get_status_routine(StatusData_t *pStatusData, JoystickReferences_t *pJoyRef, AdcVlaues_t *pAdcVals)
{
  /**
    * @note Read adc values 
    */
  Flag.AdcDataReady = 0;
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)AdcBuffer, ADC_BUFFER_MAX_SIZE);
  while (Flag.AdcDataReady == 0);
  
  /**
    * @note calculate Joystick potentiometers position(XYZ axis) (0-200).
    *       y = (((y1 - y0)/(x1 - x0)) * (x - x0)) + y0
    *       for negative position: x0 = ADC_VALUE_LOWER_LIMIT                    y0 = NegativeMin
    *                              x1 = pJoyRef->XjoyRef - ADC_REF_OFFSET        y1 = NegativeMax
    *       for positive position: x0 = (pJoyRef->XjoyRef + ADC_REF_OFFSET)      y0 = PositiveMin
    *                              x1 = ADC_VALUE_UPPER_LIMIT                    y1 = PositiveMax                
    */
  if ((pAdcVals->Xpos - (pJoyRef->XjoyRef + ADC_REF_OFFSET)) > 0)
  {
    pStatusData->Xjoy = (uint8_t)( ((float)(((float)(PositiveMax - PositiveMin)) * ((float)(pAdcVals->Xpos - (pJoyRef->XjoyRef + ADC_REF_OFFSET)))) /
                                   ((float)(ADC_VALUE_UPPER_LIMIT - (pJoyRef->XjoyRef + ADC_REF_OFFSET)))) + ((float) PositiveMin) );
  }
  else if ((pAdcVals->Xpos - (pJoyRef->XjoyRef - ADC_REF_OFFSET)) < 0)
  {
    pStatusData->Xjoy = (uint8_t)( ((float)(((float)(NegativeMax - NegativeMin)) * ((float)(pAdcVals->Xpos - ADC_VALUE_LOWER_LIMIT))) /
                                   ((float)((pJoyRef->XjoyRef - ADC_REF_OFFSET) - ADC_VALUE_LOWER_LIMIT))) + ((float) NegativeMin) );
  }
  else
  {
    pStatusData->Xjoy = 100;
  }
      
  if ((pAdcVals->Ypos - (pJoyRef->YjoyRef + ADC_REF_OFFSET)) > 0)
  {
    pStatusData->Yjoy = (uint8_t)( ((float)(((float)(PositiveMax - PositiveMin)) * ((float)(pAdcVals->Ypos - (pJoyRef->YjoyRef + ADC_REF_OFFSET)))) /
                                   ((float)(ADC_VALUE_UPPER_LIMIT - (pJoyRef->YjoyRef + ADC_REF_OFFSET)))) + ((float) PositiveMin) );
  }
   
  else if ((pAdcVals->Ypos - (pJoyRef->YjoyRef - ADC_REF_OFFSET)) < 0)
  {
    pStatusData->Yjoy = (uint8_t)( ((float)(((float)(NegativeMax - NegativeMin)) * ((float)(pAdcVals->Ypos - ADC_VALUE_LOWER_LIMIT))) /
                                   ((float)((pJoyRef->YjoyRef - ADC_REF_OFFSET) - ADC_VALUE_LOWER_LIMIT))) + ((float) NegativeMin) );
  }
  else
  {
    pStatusData->Yjoy = 100;
  }
   
  if ((pAdcVals->Zpos - (pJoyRef->ZjoyRef + ADC_REF_OFFSET)) > 0)
  {
    pStatusData->Zjoy = (uint8_t)( ((float)(((float)(PositiveMax - PositiveMin)) * ((float)(pAdcVals->Zpos - (pJoyRef->ZjoyRef + ADC_REF_OFFSET)))) /
                                   ((float)(ADC_VALUE_UPPER_LIMIT - (pJoyRef->ZjoyRef + ADC_REF_OFFSET)))) + ((float) PositiveMin) );
  }
   
  else if ((pAdcVals->Zpos - (pJoyRef->ZjoyRef - ADC_REF_OFFSET)) < 0)
  {
    pStatusData->Zjoy = (uint8_t)( ((float)(((float)(NegativeMax - NegativeMin)) * ((float)(pAdcVals->Zpos - ADC_VALUE_LOWER_LIMIT))) /
                                   ((float)((pJoyRef->ZjoyRef - ADC_REF_OFFSET) - ADC_VALUE_LOWER_LIMIT))) + ((float) NegativeMin) );
  } 
  else
  {
    pStatusData->Zjoy = 100;
  }
  
  /**
    * @note calculate Speed potentiometer position (0-10).
    *       y = (((y1 - y0)/(x1 - x0)) * (x - x0)) + y0
    *       x0 = ADC_VALUE_LOWER_LIMIT        y0 = SpeedMin
    *       x1 = ADC_VALUE_UPPER_LIMIT        y1 = SpeedMax  
    */
  pStatusData->Speed = (uint8_t) round( ((float)(((float)(SpeedMax - SpeedMin)) * ((float)(pAdcVals->Spos - ADC_VALUE_LOWER_LIMIT))) /
                                        ((float)(ADC_VALUE_UPPER_LIMIT - ADC_VALUE_LOWER_LIMIT))) + ((float) SpeedMin) );
  
  /**
    * @note Read Buttons situation from I2C (First MCP23017 , Second MCP23017)
    */
  MCP23017_read_gpio(&hmcp2, MCP23017_PORTB);
  pStatusData->Keypad2 = hmcp2.Gpio[MCP23017_PORTB];
  
  MCP23017_read_gpio(&hmcp1, MCP23017_PORTB);
  pStatusData->Keypad1 = hmcp1.Gpio[MCP23017_PORTB];
}

/**
  * @brief   In this function routine of Led command is performed. 
  *          This function writes the values of the Leds to I2C (First MCP23017 , Second MCP23017).
  *          Based on the Command Data, the status of the Leds and the buzzer is determined
  * @param   pDataAndCmd specifies the pointer to Data And Command structure (receive commands and the received Command Data from Uart).
  * @returns None
  * @note    None
  */
void led_status_routine(DataAndCommand_t *pDataAndCmd)
{
  /**
    * @note Detrermine the status of First MCP23017 Leds Based on the Command Data.
    */
  hmcp1.Gpio[MCP23017_PORTA] = pDataAndCmd->CmdData[0];
  MCP23017_write_gpio(&hmcp1, MCP23017_PORTA);
  
  /**
    * @note Detrermine the status of Second MCP23017 Leds Based on the Command Data.
    */
  hmcp2.Gpio[MCP23017_PORTA] = pDataAndCmd->CmdData[1];
  MCP23017_write_gpio(&hmcp2, MCP23017_PORTA);
  
  /**
    * @note Detrermine the status of Buzzer Based on the Command Data.
    */
  if (pDataAndCmd->CmdData[2] != 0x00)
  {
    buzzer(50);
  }
}

/**
  * @brief   This function turns the buzzer on for BeepTime milliseconds and then turns it off.
  * @param   BeepTime Specify the time that the buzzer is turn on.
  * @returns None
  * @note    None
  */
void buzzer (uint16_t BeepTime)
{
  /*Turn On the buzzer */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  delay_ms(BeepTime);
  /*Turn Off the buzzer */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
