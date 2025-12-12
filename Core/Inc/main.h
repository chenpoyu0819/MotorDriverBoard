/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define STEP_PIN_Pin          GPIO_PIN_1
#define STEP_PIN_GPIO_Port    GPIOA
#define DIR_PIN_Pin           GPIO_PIN_2
#define DIR_PIN_GPIO_Port     GPIOA
#define ENABLE_PIN_Pin        GPIO_PIN_3
#define ENABLE_PIN_GPIO_Port  GPIOA
#define SPI_CS_Pin            GPIO_PIN_4
#define SPI_CS_GPIO_Port      GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
