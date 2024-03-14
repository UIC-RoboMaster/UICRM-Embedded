/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RM_RTOS_Init(void);
void RM_RTOS_Mutexes_Init(void);
void RM_RTOS_Semaphores_Init(void);
void RM_RTOS_Timers_Init(void);
void RM_RTOS_Queues_Init(void);
void RM_RTOS_Threads_Init(void);
void RM_RTOS_Ready(void);
void RM_RTOS_Default_Task(const void *argument);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_EN1_Pin GPIO_PIN_13
#define POWER_EN1_GPIO_Port GPIOC
#define POWER_EN2_Pin GPIO_PIN_14
#define POWER_EN2_GPIO_Port GPIOC
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define Accel_CS_Pin GPIO_PIN_0
#define Accel_CS_GPIO_Port GPIOC
#define Gyro_CS_Pin GPIO_PIN_3
#define Gyro_CS_GPIO_Port GPIOC
#define WS2812B_SPI_Pin GPIO_PIN_7
#define WS2812B_SPI_GPIO_Port GPIOA
#define DCMI_PWRDOWN_Pin GPIO_PIN_5
#define DCMI_PWRDOWN_GPIO_Port GPIOC
#define IMU_TEMP_Pin GPIO_PIN_1
#define IMU_TEMP_GPIO_Port GPIOB
#define Accel_INT_Pin GPIO_PIN_10
#define Accel_INT_GPIO_Port GPIOE
#define Accel_INT_EXTI_IRQn EXTI15_10_IRQn
#define Gyro_INT_Pin GPIO_PIN_12
#define Gyro_INT_GPIO_Port GPIOE
#define Gyro_INT_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI15_10_IRQn
#define DBUS_Pin GPIO_PIN_2
#define DBUS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define RAM_D1 __attribute__((section("._D1_Area")))
#define RAM_D2 __attribute__((section("._D2_Area")))
#define RAM_D3 __attribute__((section("._D3_Area")))
#define RAM_ITC __attribute__((section("._ITCMRAM_Area")))

#define DM_MC02_GENERAL
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
