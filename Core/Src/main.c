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

#include "main.h"
#include "cmsis_os.h"

/* Private Macros */
/* For Stepper motor control signals */
#define DWT_CONTROL   (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT    (*(volatile uint32_t*)0xE0001004)
#define SCB_DEMCR     (*(volatile uint32_t*)0xE000EDFC)

SPI_HandleTypeDef hspi1;

/* Definations for Task Handlers */
/* Definitions for MotorControl */
osThreadId_t MotorControlHandle;
const osThreadAttr_t MotorControl_attributes = {
  .name = "MotorControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for PositionFeedbac */
osThreadId_t PositionFeedbacHandle;
const osThreadAttr_t PositionFeedbac_attributes = {
  .name = "PositionFeedbac",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for StepGenerator */
osThreadId_t StepGeneratorHandle;
const osThreadAttr_t StepGenerator_attributes = {
  .name = "StepGenerator",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for CommandInterfac */
osThreadId_t CommandInterfacHandle;
const osThreadAttr_t CommandInterfac_attributes = {
  .name = "CommandInterfac",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for g_MotorSpeedQueue */
osMessageQueueId_t g_MotorSpeedQueueHandle;
const osMessageQueueAttr_t g_MotorSpeedQueue_attributes = {
  .name = "g_MotorSpeedQueue"
};

/* Definitions for g_PositionMutex */
osMutexId_t g_PositionMutexHandle;
const osMutexAttr_t g_PositionMutex_attributes = {
  .name = "g_PositionMutex"
};

/* Private Variables */
volatile int32_t g_CurrentPosition    = 0;    ///< current motor position
volatile int32_t g_TargetPosition     = 0;    ///< target, for motor control

uint8_t spi_tx_dummy[2]   = {0xFF, 0xFF};   ///< sending trash message to slave(if needed)
uint8_t spi_rx_data[2]    = {0x00, 0x00};   ///< real time position info

/* Private Functions */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartMotorControl(void *argument);
void StartPositionFeedback(void *argument);
void StartStepGenerator(void *argument);
void StartCommandInterface(void *argument);
void DWT_Init(void);
void DWT_Delay_us(uint32_t us);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Init Mutex */
  g_PositionMutexHandle = osMutexNew(&g_PositionMutex_attributes);

  /* Init Message Queue */
  g_MotorSpeedQueueHandle = osMessageQueueNew (5, sizeof(int16_t), &g_MotorSpeedQueue_attributes);

  /* Init os tasks */
  MotorControlHandle = osThreadNew(StartMotorControl, NULL, &MotorControl_attributes);
  PositionFeedbacHandle = osThreadNew(StartPositionFeedback, NULL, &PositionFeedbac_attributes);
  StepGeneratorHandle = osThreadNew(StartStepGenerator, NULL, &StepGenerator_attributes);
  CommandInterfacHandle = osThreadNew(StartCommandInterface, NULL, &CommandInterfac_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_PIN_Pin|DIR_PIN_Pin|ENABLE_PIN_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_PIN_Pin DIR_PIN_Pin ENABLE_PIN_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = STEP_PIN_Pin|DIR_PIN_Pin|ENABLE_PIN_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  Function implementing the MotorControl thread.
  * @param  argument: Not used
  * @retval None
  */
void StartMotorControl(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

/**
* @brief Function implementing the PositionFeedbac thread.
* @param argument: Not used
* @retval None
*/
void StartPositionFeedback(void *argument)
{
  /* buffer for SPI position data */ 
  uint16_t rawPosition = 0;
  int32_t finalPosition = 0;    ///< may have negative value after position transform

  for(;;)
  {
    /* Chip select for SPI, pull low gpio pin */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

    /* Data interaction */
    if (HAL_SPI_TransmitReceive_IT(&hspi1, spi_tx_dummy, spi_rx_data, 2, 5) == HAL_OK)    ///< 5ms means timeout, avoid hardware error break whole rtos
    {
      /* Reassembly position data : rx_data[0] + rx_data[1] (Big endian) */
      rawPosition = (spi_rx_data[0] << 8) | spi_rx_data[1];

      /* casting data. !!warning!! need to modify based on encoder */
      finalPosition = (int32_t)(int16_t)rawPosition;    ///< gen by Gemini
    }

    if (osMutexAcquire(g_PositionMutexHandle, osWaitForever) == osOK) {
      g_CurrentPosition = finalPosition;
      osMutexRelease(g_PositionMutexHandle);
    }
    osDelay(5);
  }
}

/**
* @brief Function implementing the StepGenerator thread.
* @param argument: Not used、
* @retval None
*/
void StartStepGenerator(void *argument)
{
  /* Local Variables */
  int32_t   speedCmd    = 0;
  uint32_t  pulseDelay  = 0;
  osStatus_t status;

  for(;;)
  {
    /* Wait until new speed command from Motor control Handler */
    status = osMessageQueueGet(g_MotorSpeedQueueHandle, &speedCmd, NULL, osWaitForever);
    
    if (status == osOK) {
      while (1) {
        /* Set rotation direction */
        if (speedCmd > 0) {
          HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET);
        }

        /* Stop Handler */
        if (speedCmd == 0) {
          break;      ///< get back to message queue, wait for new command
        }

        /* speed protection & calculate pulse delay */
        int32_t absSpeed = (speedCmd > 0) ? speedCmd : -speedCmd;
        if (absSpeed > 20000) absSpeed = 20000;
        if (absSpeed < 1) absSpeed = 1;

        pulseDelay = 1000000 / absSpeed;

        /* Generate pulse signal */
        HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_SET);
        DWT_Delay_us(5);
        HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET);

        /* timeout = 0, not polling but wait until queue */
        int32_t newCmd;
        if (osMessageQueueGet(g_MotorSpeedQueueHandle, &speedCmd, NULL, 0) == osOK) {
          speedCmd = newCmd;
        }
      }
    }
  }
}

/**
* @brief Function implementing the CommandInterfac thread.
* @param argument: Not used
* @retval None
*/
void StartCommandInterface(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

/**
* @brief DWT init function for stepper motor.
* @param argument: Not used
* @retval None
*/
void DWT_Init(void)
{
    SCB_DEMCR   |= 0x01000000;
    DWT_CYCCNT  = 0;
    DWT_CONTROL |= 1;
}

/**
* @brief Delay function for stepper motor.
* @param argument: time in microseconds
* @retval None
*/
void DWT_Delay_us(uint32_t us)
{
    uint32_t startTick  = DWT_CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while (DWT_CYCCNT - startTick < delayTicks);
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
  if (htim->Instance == TIM1)
  {
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
