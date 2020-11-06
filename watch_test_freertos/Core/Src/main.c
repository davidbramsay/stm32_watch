/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "er_oled.h"
#include "dotstar.h"
#include "iqs263.h"

#define NUM_PIXELS 12
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* Definitions for screenUpdate */
osThreadId_t screenUpdateHandle;
const osThreadAttr_t screenUpdate_attributes = {
  .name = "screenUpdate",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for LEDControl */
osThreadId_t LEDControlHandle;
const osThreadAttr_t LEDControl_attributes = {
  .name = "LEDControl",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for buttonPress */
osThreadId_t buttonPressHandle;
const osThreadAttr_t buttonPress_attributes = {
  .name = "buttonPress",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for vibrateControl */
osThreadId_t vibrateControlHandle;
const osThreadAttr_t vibrateControl_attributes = {
  .name = "vibrateControl",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for rtcSecondTick */
osThreadId_t rtcSecondTickHandle;
const osThreadAttr_t rtcSecondTick_attributes = {
  .name = "rtcSecondTick",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for bleTX */
osThreadId_t bleTXHandle;
const osThreadAttr_t bleTX_attributes = {
  .name = "bleTX",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for bleRX */
osThreadId_t bleRXHandle;
const osThreadAttr_t bleRX_attributes = {
  .name = "bleRX",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for LEDTimer */
osThreadId_t LEDTimerHandle;
const osThreadAttr_t LEDTimer_attributes = {
  .name = "LEDTimer",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for touchRead */
osThreadId_t touchReadHandle;
const osThreadAttr_t touchRead_attributes = {
  .name = "touchRead",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for bleTXqueue */
osMessageQueueId_t bleTXqueueHandle;
const osMessageQueueAttr_t bleTXqueue_attributes = {
  .name = "bleTXqueue"
};
/* Definitions for rtcMutex */
osMutexId_t rtcMutexHandle;
const osMutexAttr_t rtcMutex_attributes = {
  .name = "rtcMutex"
};
/* Definitions for screenTextMutex */
osMutexId_t screenTextMutexHandle;
const osMutexAttr_t screenTextMutex_attributes = {
  .name = "screenTextMutex"
};
/* Definitions for ledStateMutex */
osMutexId_t ledStateMutexHandle;
const osMutexAttr_t ledStateMutex_attributes = {
  .name = "ledStateMutex"
};
/* USER CODE BEGIN PV */


typedef enum {
	LED_NONE,
	LED_OFF,
	LED_TIME,
	LED_TOUCH_TRACK,
	LED_CONFIRM_FLASH,
	LED_SPIRAL,
	LED_OTHER
} LedStatus_t;

typedef struct {
	LedStatus_t currentMode;
	LedStatus_t nextMode;
	TickType_t modeTimeout;
} LedState_t;

LedState_t LedState;
//protected by ledStateMutex

typedef enum {
	SCREEN_OFF,
	SCREEN_TIME,
	SCREEN_TOUCH_TRACK,
	SCREEN_TEXT,
	SCREEN_IMAGE
} ScreenStatus_t;

typedef struct {
	char screenText[128];
	uint8_t screenImage;
	uint8_t len;
} ScreenState_t;

ScreenState_t ScreenState;
//protected by screenTextMutex

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RF_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
void startScreenUpdate(void *argument);
void startLEDControl(void *argument);
void startButtonPress(void *argument);
void startVibrateControl(void *argument);
void startRTCTick(void *argument);
void startBLETX(void *argument);
void startBLERX(void *argument);
void startLEDTimer(void *argument);
void startTouchRead(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RF_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of rtcMutex */
  rtcMutexHandle = osMutexNew(&rtcMutex_attributes);

  /* creation of screenTextMutex */
  screenTextMutexHandle = osMutexNew(&screenTextMutex_attributes);

  /* creation of ledStateMutex */
  ledStateMutexHandle = osMutexNew(&ledStateMutex_attributes);

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
  /* creation of bleTXqueue */
  bleTXqueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &bleTXqueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of screenUpdate */
  screenUpdateHandle = osThreadNew(startScreenUpdate, NULL, &screenUpdate_attributes);

  /* creation of LEDControl */
  LEDControlHandle = osThreadNew(startLEDControl, NULL, &LEDControl_attributes);

  /* creation of buttonPress */
  buttonPressHandle = osThreadNew(startButtonPress, NULL, &buttonPress_attributes);

  /* creation of vibrateControl */
  vibrateControlHandle = osThreadNew(startVibrateControl, NULL, &vibrateControl_attributes);

  /* creation of rtcSecondTick */
  rtcSecondTickHandle = osThreadNew(startRTCTick, NULL, &rtcSecondTick_attributes);

  /* creation of bleTX */
  bleTXHandle = osThreadNew(startBLETX, NULL, &bleTX_attributes);

  /* creation of bleRX */
  bleRXHandle = osThreadNew(startBLERX, NULL, &bleRX_attributes);

  /* creation of LEDTimer */
  LEDTimerHandle = osThreadNew(startLEDTimer, NULL, &LEDTimer_attributes);

  /* creation of touchRead */
  touchReadHandle = osThreadNew(startTouchRead, NULL, &touchRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */


  RCC_OscInitTypeDef RCC_OscInitLSE;

  RCC_OscInitLSE.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitLSE.LSEState = RCC_LSE_ON;

  if(HAL_RCC_OscConfig(&RCC_OscInitLSE) != HAL_OK){
        Error_Handler();
  }

  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OLED_RESET_Pin */
  GPIO_InitStruct.Pin = OLED_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

static inline void set_bit(long *x, int bitNum) {
    *x |= (1L << bitNum);
}

static inline void clear_bit(long *x, int bitNum) {
    *x &= (~(1L << bitNum));
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(buttonPressHandle, GPIO_Pin, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}


uint64_t get_RTC_full(){ //unverified
    RTC_TimeTypeDef cTime;
    RTC_DateTypeDef cDate;

    osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
    osMutexRelease(rtcMutexHandle);

    uint64_t full_rtc_val = (cDate.WeekDay << (8*3)) | (cDate.Month << (8*2)) | (cDate.Date << (8*1)) | cDate.Year;
    full_rtc_val <<= 32;
    full_rtc_val |= (cTime.Hours << (8*3)) | (cTime.Minutes << (8*2)) | (cTime.Seconds << (8*1)) | (cTime.TimeFormat);

    return full_rtc_val;

}


void get_RTC_hrmin(char *dest) {

    RTC_TimeTypeDef cTime;
    RTC_DateTypeDef cDate;

    osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
    //must get date as well; RTC shadow registers will error if both aren't accessed
    HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
    osMutexRelease(rtcMutexHandle);

    uint8_t  hrs = RTC_Bcd2ToByte(cTime.Hours);
    uint8_t mins = RTC_Bcd2ToByte(cTime.Minutes);

    char time[5];
    sprintf (time, "%02d%02d", hrs, mins);

    strncpy(dest, time, sizeof(time));

}


void get_RTC_hrminsec(char *dest) {

	RTC_TimeTypeDef cTime;
    RTC_DateTypeDef cDate;


	osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
	HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
	//must get date as well; RTC shadow registers will error if both aren't accessed
	HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
	osMutexRelease(rtcMutexHandle);

	uint8_t  hrs = RTC_Bcd2ToByte(cTime.Hours);
	uint8_t mins = RTC_Bcd2ToByte(cTime.Minutes);
	uint8_t secs = RTC_Bcd2ToByte(cTime.Seconds);

	char time[10];
	sprintf (time, "%02d:%02d:%02d", hrs, mins, secs);

	strncpy(dest, time, sizeof(time));

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startScreenUpdate */
/**
  * @brief  Function implementing the screenUpdate thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startScreenUpdate */
void startScreenUpdate(void *argument)
{
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_SET);

  uint8_t oled_buf[WIDTH * HEIGHT / 8];

  er_oled_begin();
  er_oled_clear(oled_buf);
  er_oled_string(6, 14, "DRAMSAY", 12, 1, oled_buf);
  er_oled_display(oled_buf);

  osDelay(3000);
  er_oled_clear(oled_buf);
  er_oled_display(oled_buf);


  ScreenStatus_t screenStatus= SCREEN_TIME;
  char screenText[128];
  char time[10];
  uint8_t imageNum;

  /* Infinite loop */
  for(;;)
  {

	  	//wait for someone to update screen state elsewhere and notify
	  	xTaskNotifyWait(0x00, 0x00, &screenStatus, portMAX_DELAY);
	  	switch(screenStatus){

	  			case SCREEN_TIME:
	  				//test time
	  				//er_oled_time("1743");

	  				//only hrmin
	  				get_RTC_hrmin(time);
	  				er_oled_time(time);

	  				//hrminsec
	  				//get_RTC_hrminsec(time);
	  				//er_oled_clear(oled_buf);
	  				//er_oled_string(0, 14, time, 12, 1, oled_buf);
	  				//er_oled_display(oled_buf);

	  				break;

	  			case SCREEN_TOUCH_TRACK:

	  				break;

	  			case SCREEN_IMAGE:

	  				er_oled_clear(oled_buf);

	  				osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
	  				imageNum = ScreenState.screenImage;
	  				osMutexRelease(screenTextMutexHandle);

	  				if (imageNum == 1){er_oled_bitmap(0, 0, PIC1, 72, 40, oled_buf);}
	  				else if (imageNum == 2) {er_oled_bitmap(0, 0, PIC2, 72, 40, oled_buf);}
	  				else {er_oled_string(0, 14, "invalid image number", 24, 1, oled_buf);}

	  				er_oled_display(oled_buf);
	  				osDelay(1000);
	  				command(0xa7);//--set Negative display
	  				osDelay(1000);
	  				command(0xa6);//--set normal display
	  				break;

	  			case SCREEN_TEXT:

	  				osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
	  				strncpy(screenText, ScreenState.screenText, sizeof(ScreenState.screenText));
	  				osMutexRelease(screenTextMutexHandle);
	  				er_oled_clear(oled_buf);
	  				er_oled_string(0, 14, screenText, 12, 1, oled_buf);
	  				er_oled_display(oled_buf);
	  				osDelay(5);
	  				break;

	  			default: //includes SCREEN_OFF
	  				er_oled_clear(oled_buf);
	  				er_oled_display(oled_buf);
	  				break;
	  	}

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startLEDControl */
/**
* @brief Function implementing the LEDControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startLEDControl */
void startLEDControl(void *argument)
{
  /* USER CODE BEGIN startLEDControl */

  //To call LED behavior:
    //osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
   	//LedState.currentMode = LED_CONFIRM_FLASH;
   	//osMutexRelease(ledStateMutexHandle);


  //LedState Init
  osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
  LedState.currentMode = LED_SPIRAL;
  LedState.nextMode = LED_OFF;
  LedState.modeTimeout = pdMS_TO_TICKS(5000);
  osMutexRelease(ledStateMutexHandle);

  //Dotstar Init
  DotStar_InitHandle dotstar;
  dotstar.spiHandle = &hspi1;
  dotstar.numLEDs = NUM_PIXELS;
  dotstar.colorOrder = DOTSTAR_BGR;
  Dotstar_Init(&dotstar);

  ds_clear();  //turn off
  ds_show();

  const uint8_t STANDARD_BRIGHTNESS = 20; //20, 0-255
  const uint8_t MAX_BRIGHTNESS = 0x33; //max brightness, 0x01-0xFF

  ds_setBrightness(STANDARD_BRIGHTNESS);
  osDelay(1000);

  LedStatus_t currentMode;
  LedStatus_t lastLoopMode = LED_OFF;

  uint16_t counter = 0;
  uint8_t stateVar1 = 0;
  uint8_t stateVar2 = 0;

  uint32_t color = 0x000000;
  /* Infinite loop */
  for(;;)
  {
    //check state, get mode, call timer if necessary

    osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
	currentMode = LedState.currentMode;
	if (LedState.modeTimeout){
		xTaskNotifyGive(LEDTimerHandle);
	}
	osMutexRelease(ledStateMutexHandle);

	//reset count if we've switched modes
	if (lastLoopMode != currentMode) { counter = 0; stateVar1 = 0; stateVar2 = 0;}

	switch(currentMode){

		case LED_TIME:

			break;
		case LED_TOUCH_TRACK:

			break;
		case LED_CONFIRM_FLASH:

			//each color go from 00 to MAX_BRIGHTNESS to 00 over a second, 1000Hz=sec, ~512 steps, 2ms
			if (lastLoopMode != currentMode) { ds_fill(0xFFFFFF, 0, 12);}
		    ds_setBrightness(stateVar1);
			ds_show();

		    //increment color intensity
		    if (stateVar2) {stateVar1--;}
		    else {stateVar1++;}

		    //if we hit a limit switch color scaling up or down
		    if (stateVar1 == MAX_BRIGHTNESS) {stateVar2 = 1;}
		    if (stateVar1 == 0x00) {stateVar2 = 0;}

			osDelay(pdMS_TO_TICKS(2)); //2ms delay

			if (++counter == (MAX_BRIGHTNESS*4)) { //if we hit 1 cycle here (= MAX_BRIGHTNESS*2,could *4 to set to two full cycles), set state to off
				ds_clear();
				ds_show();
				ds_setBrightness(STANDARD_BRIGHTNESS);

				osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
				LedState.currentMode = LED_OFF;
				osMutexRelease(ledStateMutexHandle);
			}

			break;

		case LED_SPIRAL:

			//rotate fixed pattern around 12
			//modulo 12
			for (int i=0; i< NUM_PIXELS; i++){

				if (i==(counter+2)%12){ color = 0xFFFFFF; }
				else if (i==(counter+1)%12){ color = 0xD0D0D0; }
				else if (i==counter)       { color = 0xA0A0A0; }
				else { color = 0x000000; }

				ds_setPixelColor32B(i, color); // 'off' pixel at head
			}

			ds_show();
			counter = (counter+1)%12;
			osDelay(pdMS_TO_TICKS(50));

			break;

		default: //case LED_OTHER, LED_OFF, LED_NONE
			if (lastLoopMode != currentMode) {
				ds_clear();
				ds_show();
			}
			osDelay(250);
			break;
	}

	lastLoopMode = currentMode;
  }
  /* USER CODE END startLEDControl */
}

/* USER CODE BEGIN Header_startButtonPress */
/**
* @brief Function implementing the buttonPress thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startButtonPress */
void startButtonPress(void *argument)
{
  /* USER CODE BEGIN startButtonPress */
  /* Infinite loop */

  //Buttons are PULLED UP and drop to 0 when pressed
  uint8_t buttonState[] = {1, 1, 1};
  uint32_t callingPin = 0x00;

  for(;;)
  {
	//wait for rising or falling edge trigger, put calling pin in callingPin
	xTaskNotifyWait(0x00, 0x00, &callingPin, portMAX_DELAY);

	//check state of pin
	GPIO_PinState first_read = HAL_GPIO_ReadPin(GPIOB, callingPin);

	//wait 50ms
    osDelay(50);

    //check again (debounce) to get a good reading
	if (first_read == HAL_GPIO_ReadPin(GPIOB, callingPin)){

		//check that we are actually changing state; the edge trigger should only call
		//when this happens (except during debouncing) so we expect this to be true
		//almost always

		//callingPin can be used as bitmask Pin 5/4/3 give 1000000/10000/1000

		if (callingPin == 0b1000 && first_read != buttonState[0]) { //button 1 trigger
		  //set buttonState
		  buttonState[0] = first_read;

		  //do stuff if button pressed
		  if (!first_read){
		       	osDelay(100);
		  }

		  		    //send BLE queue indicator; button 1 = 0x0
		  		    uint16_t bleval = 0x0000 | ((!first_read) << 8);
		  		    osMessageQueuePut(bleTXqueueHandle, &bleval, 0, 0);
		}
		if (callingPin == 0b10000 && first_read != buttonState[1]) { //button 2 trigger
		    //set buttonState
		    buttonState[1] = first_read;

		    //do stuff if button pressed
		    if (!first_read){
		       	osDelay(100);
		    }

		    //send BLE queue indicator; button 2 = 0x1
		    uint16_t bleval = 0x1000 | ((!first_read) << 8);
		    osMessageQueuePut(bleTXqueueHandle, &bleval, 0, 0);
		}
		if (callingPin == 0b100000 && first_read != buttonState[2]) { //button 3 trigger
		    //set buttonState
		    buttonState[2] = first_read;

		    //do stuff if button pressed
		    if (!first_read){
		    	osDelay(100);
		    }

		    //send BLE queue indicator; button 3 = 0x2
		    uint16_t bleval = 0x2000 | ((!first_read) << 8);
		    osMessageQueuePut(bleTXqueueHandle, &bleval, 0, 0);
		}

	}

  }
  /* USER CODE END startButtonPress */
}

/* USER CODE BEGIN Header_startVibrateControl */
/**
* @brief Function implementing the vibrateControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startVibrateControl */
void startVibrateControl(void *argument)
{
  /* USER CODE BEGIN startVibrateControl */

  //HAL_GPIO_WritePin(VIBRATION_GPIO_Port, VIBRATION_Pin, GPIO_PIN_RESET);

  //Init Vibration Motor PWM Parameters
  int duty_cycle = 79; //0 is off, up to ~80
  htim1.Instance->CCR2 = duty_cycle;

  /* Infinite loop */
  for(;;)
  {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    osDelay(2000);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    osDelay(2000);

  }
  /* USER CODE END startVibrateControl */
}

/* USER CODE BEGIN Header_startRTCTick */
/**
* @brief Function implementing the rtcSecondTick thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startRTCTick */
void startRTCTick(void *argument)
{
  /* USER CODE BEGIN startRTCTick */

	RTC_TimeTypeDef sTime = {0};
    sTime.Hours      = 0x15;
    sTime.Minutes    = 0x41;
    sTime.Seconds    = 0x57;
    sTime.SubSeconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
   	sTime.StoreOperation = RTC_STOREOPERATION_RESET;

   	osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
	    Error_Handler();
	}
	osMutexRelease(rtcMutexHandle);

  /* Infinite loop */
  for(;;)
  {
    osDelay(6000);

    osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
    LedState.currentMode = LED_SPIRAL;
    osMutexRelease(ledStateMutexHandle);

    osDelay(3000);

    ScreenStatus_t newScreen = SCREEN_TIME;
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);

    osDelay(3000);

    osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
    LedState.currentMode = LED_CONFIRM_FLASH;
    osMutexRelease(ledStateMutexHandle);

    osDelay(1000);

    osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
    LedState.currentMode = LED_SPIRAL;
    osMutexRelease(ledStateMutexHandle);

    osDelay(3000);

    newScreen = SCREEN_TEXT;
	osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
	strncpy(ScreenState.screenText, "arbitrary", sizeof("arbitrary"));
	osMutexRelease(screenTextMutexHandle);
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);

    osDelay(3000);

    /*
    //set rtc
    RTC_TimeTypeDef sTime = {0};
    sTime.Hours      = 0x15;
    sTime.Minutes    = 0x41;
    sTime.Seconds    = 0x57;
    sTime.SubSeconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
   	sTime.StoreOperation = RTC_STOREOPERATION_RESET;

   	osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }
    osMutexRelease(rtcMutexHandle);
	*/

    newScreen = SCREEN_TIME;
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);
    osDelay(1000);
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);
    osDelay(1000);
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);
    osDelay(1000);
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);
    osDelay(1000);


    osDelay(1000);

    newScreen = SCREEN_IMAGE;
   	osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
   	ScreenState.screenImage = 1;
   	osMutexRelease(screenTextMutexHandle);
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);

    osDelay(3000);

    newScreen = SCREEN_OFF;
    xTaskNotify(screenUpdateHandle, (uint32_t)newScreen, eSetValueWithOverwrite);


  }
  /* USER CODE END startRTCTick */
}

/* USER CODE BEGIN Header_startBLETX */
/**
* @brief Function implementing the bleTX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBLETX */
void startBLETX(void *argument)
{
  /* USER CODE BEGIN startBLETX */
  /* Infinite loop */
  for(;;)
  {
    osDelay(2000);

  }
  /* USER CODE END startBLETX */
}

/* USER CODE BEGIN Header_startBLERX */
/**
* @brief Function implementing the bleRX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBLERX */
void startBLERX(void *argument)
{
  /* USER CODE BEGIN startBLERX */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startBLERX */
}

/* USER CODE BEGIN Header_startLEDTimer */
/**
* @brief Function implementing the LEDTimer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startLEDTimer */
void startLEDTimer(void *argument)
{
  /* USER CODE BEGIN startLEDTimer */

  //this gets called if the timer value in the struct is
  //set to something other than 0 (which should occur using
  //the ledSateMutexHandle to make it atomic).  It will
  //change the currentState to nextState after the timeout
  //in ms IF no state updates have occurred during the waiting
  //period.  Changing current or next state will cause the
  //this timer to stop working.

  //Updating the timeout value when another timeout is active
  //has poorly defined behavior; namely it will wait for the
  //active timeout to finish before it starts counting.

  //This should really be used sparingly to simply call a
  //temporary LED action that should then just fade to
  //another, like a confirm flash followed by off/watch mode.

  LedState_t waitState;

  /* Infinite loop */
  for(;;)
  {
	  //wait until notified
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY);

	  //pull time of delay before updating LED state
	  osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
	  waitState = LedState;
	  LedState.modeTimeout = 0;
	  osMutexRelease(ledStateMutexHandle);

	  //delay
	  osDelay(waitState.modeTimeout);

	  //update LED state in LedState
	  osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
	  //check that state values haven't changed since
	  //started waiting before updating state
	  if (waitState.currentMode == LedState.currentMode && waitState.nextMode == LedState.nextMode) {
		  LedState.currentMode = LedState.nextMode;
		  LedState.nextMode = LED_NONE;
	  }
	  osMutexRelease(ledStateMutexHandle);
  }
  /* USER CODE END startLEDTimer */
}

/* USER CODE BEGIN Header_startTouchRead */
/**
* @brief Function implementing the touchRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTouchRead */
void startTouchRead(void *argument)
{
  /* USER CODE BEGIN startTouchRead */

  int16_t current_minute = -1;
  uint8_t touch_end_count = 0;
  uint16_t last_minute = -1;
  #define TOUCH_END_TIMEOUT 6

  osDelay(3000); // give screen time to turn on.

  //init peripheral (not turbo mode, poll every 250ms, if touch sample at 40Hz until no touch)
  if (setup_iqs263() == HAL_ERROR) {

	  osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
	  strncpy(ScreenState.screenText, "touch fail", sizeof("touch fail"));
	  osMutexRelease(screenTextMutexHandle);
	  xTaskNotify(screenUpdateHandle, (uint32_t)SCREEN_TEXT, eSetValueWithOverwrite);

  }

  /* Infinite loop */
  for(;;)
  {
   current_minute = iqs263_get_min_if_pressed(); //returns -1 if no press
   if (current_minute != -1) { //touch!

	   touch_end_count = 1;

	   if (last_minute != current_minute) {
		   //update touch stuff!
		   last_minute = current_minute;
	   	   er_oled_print_2digit(current_minute);
	   }

	   //optional
	   osDelay(25);

	   /*
	   char str[3];
	   sprintf(str, "%d", current_minute);
	   osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
	   strncpy(ScreenState.screenText, str, sizeof(str));
	   osMutexRelease(screenTextMutexHandle);
	   xTaskNotify(screenUpdateHandle, (uint32_t)SCREEN_TEXT, eSetValueWithOverwrite);
	   */

   } else if (touch_end_count > 0){

	   touch_end_count += 1;//increment touching_end_count

	   if (touch_end_count >= TOUCH_END_TIMEOUT){  //if it hits this value, we're done

		   touch_end_count = 0;

		   //DO THINGS WITH CONFIRMED TOUCH == LAST_MINUTE
		   osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
		   char out_text[10];
		   sprintf(out_text, "FINAL: %d", last_minute);
		   strncpy(ScreenState.screenText, out_text, sizeof(out_text));
		   osMutexRelease(screenTextMutexHandle);
		   xTaskNotify(screenUpdateHandle, (uint32_t)SCREEN_TEXT, eSetValueWithOverwrite);

		   last_minute = -1;

	   }

	   osDelay(25);


   }else { //no touch, wait for a touch
    osDelay(250);
   }
  }
  /* USER CODE END startTouchRead */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
