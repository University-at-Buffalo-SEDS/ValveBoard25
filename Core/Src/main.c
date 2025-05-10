/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "Drivers/ltc2990.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float voltages[2];
  uint32_t pressure;
} valveBoardPayload_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

/* Definitions for blinkLED */
osThreadId_t blinkLEDHandle;
const osThreadAttr_t blinkLED_attributes = {
  .name = "blinkLED",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000 * 4
};
/* Definitions for readInstrumenta */
osThreadId_t readInstrumentaHandle;
const osThreadAttr_t readInstrumenta_attributes = {
  .name = "readInstrumenta",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000 * 4
};
/* Definitions for printInstrument */
osThreadId_t printInstrumentHandle;
const osThreadAttr_t printInstrument_attributes = {
  .name = "printInstrument",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000 * 4
};
/* Definitions for ADCReadTask */
osThreadId_t ADCReadTaskHandle;
const osThreadAttr_t ADCReadTask_attributes = {
  .name = "ADCReadTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000 * 4
};
/* Definitions for ADCPrintTask */
osThreadId_t ADCPrintTaskHandle;
const osThreadAttr_t ADCPrintTask_attributes = {
  .name = "ADCPrintTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1000 * 4
};
/* Definitions for sendMessage */
osThreadId_t sendMessageHandle;
const osThreadAttr_t sendMessage_attributes = {
  .name = "sendMessage",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000 * 4
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* USER CODE BEGIN PV */

LTC2990_Handle_t LTC2990_Handle;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
void startBlinkLED(void *argument);
void startReadInstrumentation(void *argument);
void StartSendMessage(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PRINT_BUFFER_SIZE     256

void CDC_Transmit_Print(const char *format, ...)
{
    char buf[PRINT_BUFFER_SIZE];
    va_list  args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len <= 0) {
        return;
    }
    if (len > (PRINT_BUFFER_SIZE - 1)) {
        len = PRINT_BUFFER_SIZE - 1;
    }
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len);
}
//uint32_t adc_data;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == EMER_VALVE_IN_Pin) {
		GPIO_PinState in = HAL_GPIO_ReadPin(EMER_VALVE_IN_GPIO_Port, EMER_VALVE_IN_Pin);
		HAL_GPIO_WritePin(Emergency_Valve_OUT_GPIO_Port, Emergency_Valve_OUT_Pin, in);
	} else if (GPIO_Pin == FC_FAULT_Pin) {
		HAL_GPIO_WritePin(Emergency_Valve_OUT_GPIO_Port, Emergency_Valve_OUT_Pin, GPIO_PIN_SET);
	} else if (GPIO_Pin == SOLENOID_IN_Pin) {
		HAL_GPIO_WritePin(Solenoid_OUT_GPIO_Port, Solenoid_OUT_Pin, GPIO_PIN_SET);
	} else {
		return;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_ADC3_Init();
  MX_FDCAN2_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_FDCAN_Start(&hfdcan2);

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (16, sizeof(valveBoardPayload_t), &sensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinkLED */
  blinkLEDHandle = osThreadNew(startBlinkLED, NULL, &blinkLED_attributes);

  /* creation of readInstrumenta */
  readInstrumentaHandle = osThreadNew(startReadInstrumentation, NULL, &readInstrumenta_attributes);

  /* creation of sendMessage */
  sendMessageHandle = osThreadNew(StartSendMessage, NULL, &sendMessage_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 16;
  hfdcan2.Init.NominalTimeSeg1 = 63;
  hfdcan2.Init.NominalTimeSeg2 = 16;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 13;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Solenoid_OUT_Pin|Emergency_Valve_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FRONT_LED_GPIO_Port, FRONT_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : EMER_VALVE_IN_Pin */
  GPIO_InitStruct.Pin = EMER_VALVE_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EMER_VALVE_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SOLENOID_IN_Pin */
  GPIO_InitStruct.Pin = SOLENOID_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SOLENOID_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FC_FAULT_Pin */
  GPIO_InitStruct.Pin = FC_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FC_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Solenoid_OUT_Pin Emergency_Valve_OUT_Pin */
  GPIO_InitStruct.Pin = Solenoid_OUT_Pin|Emergency_Valve_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FRONT_LED_Pin */
  GPIO_InitStruct.Pin = FRONT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FRONT_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* Configure PB13 as analog for ADC3_IN5 */
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startBlinkLED */
/**
  * @brief  Function implementing the blinkLED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startBlinkLED */
void startBlinkLED(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(FRONT_LED_GPIO_Port, FRONT_LED_Pin);
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startReadInstrumentation */
/**
* @brief Function implementing the readInstrumenta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startReadInstrumentation */
void startReadInstrumentation(void *argument)
{
  /* USER CODE BEGIN startReadInstrumentation */
	LTC2990_Init(&LTC2990_Handle, &hi2c2, LTC2990_I2C_ADDRESS);
  /* Infinite loop */
  for(;;)
  {
	float raw_voltages[2];
	valveBoardPayload_t payload;

	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3,20);
	payload.pressure = HAL_ADC_GetValue(&hadc3);

	LTC2990_Step(&LTC2990_Handle);

	LTC2990_Get_Single_Ended_Voltage(&LTC2990_Handle, raw_voltages);


	payload.voltages[0] = raw_voltages[0];
	payload.voltages[1] = raw_voltages[1];

	osMessageQueuePut(sensorQueueHandle, &payload, 0, osWaitForever);

	CDC_Transmit_Print("Voltage 1: %f \r\n", payload.voltages[0]);
	CDC_Transmit_Print("Voltage 2: %f \r\n", payload.voltages[1]);
	CDC_Transmit_Print("ADC Data: %d \r\n", payload.pressure);

    osDelay(50);
  }
  /* USER CODE END startReadInstrumentation */
}


/* USER CODE BEGIN Header_StartSendMessage */
/**
* @brief Function implementing the sendMessage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendMessage */
void StartSendMessage(void *argument)
{
  /* USER CODE BEGIN StartSendMessage */
  /* Infinite loop */
	valveBoardPayload_t payload;
	  uint8_t txBuf[sizeof(valveBoardPayload_t)];
	  FDCAN_TxHeaderTypeDef txHeader = {
		  .Identifier = 0x333,
		  .IdType = FDCAN_STANDARD_ID,
		  .TxFrameType = FDCAN_DATA_FRAME,
		  .DataLength = FDCAN_DLC_BYTES_12,
		  .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		  .BitRateSwitch = FDCAN_BRS_ON,
		  .FDFormat = FDCAN_FD_CAN,
		  .TxEventFifoControl = FDCAN_STORE_TX_EVENTS,
		  .MessageMarker = 0
	  };
  for(;;)
  {
	  osMessageQueueGet(sensorQueueHandle, &payload, NULL, osWaitForever);
	  memcpy(txBuf, &payload, sizeof(payload));
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txBuf);
	  osDelay(100);
  }
  /* USER CODE END StartSendMessage */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
