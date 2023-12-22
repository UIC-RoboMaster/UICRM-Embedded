/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define IST8310_DRDY_Pin GPIO_PIN_3
#define IST8310_DRDY_GPIO_Port GPIOE
#define IST8310_DRDY_EXTI_IRQn EXTI3_IRQn
#define IST8310_RSTN_Pin GPIO_PIN_2
#define IST8310_RSTN_GPIO_Port GPIOE
#define MPU6500_IT_Pin GPIO_PIN_8
#define MPU6500_IT_GPIO_Port GPIOB
#define MPU6500_IT_EXTI_IRQn EXTI9_5_IRQn
#define IMU_HEATER_Pin GPIO_PIN_5
#define IMU_HEATER_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
#define MOS_CTL1_Pin GPIO_PIN_2
#define MOS_CTL1_GPIO_Port GPIOH
#define MOS_CTL2_Pin GPIO_PIN_3
#define MOS_CTL2_GPIO_Port GPIOH
#define MOS_CTL3_Pin GPIO_PIN_4
#define MOS_CTL3_GPIO_Port GPIOH
#define LED_G8_Pin GPIO_PIN_8
#define LED_G8_GPIO_Port GPIOG
#define VCC_5V_TO_12V_ADJ_Pin GPIO_PIN_4
#define VCC_5V_TO_12V_ADJ_GPIO_Port GPIOF
#define MOS_CTL4_Pin GPIO_PIN_5
#define MOS_CTL4_GPIO_Port GPIOH
#define LED_G7_Pin GPIO_PIN_7
#define LED_G7_GPIO_Port GPIOG
#define LED_G6_Pin GPIO_PIN_6
#define LED_G6_GPIO_Port GPIOG
#define MPU6500_CS_Pin GPIO_PIN_6
#define MPU6500_CS_GPIO_Port GPIOF
#define LED_G5_Pin GPIO_PIN_5
#define LED_G5_GPIO_Port GPIOG
#define LED_G4_Pin GPIO_PIN_4
#define LED_G4_GPIO_Port GPIOG
#define LED_G3_Pin GPIO_PIN_3
#define LED_G3_GPIO_Port GPIOG
#define TRIG_KEY_Pin GPIO_PIN_10
#define TRIG_KEY_GPIO_Port GPIOF
#define TRIG_KEY_EXTI_IRQn EXTI15_10_IRQn
#define LED_G2_Pin GPIO_PIN_2
#define LED_G2_GPIO_Port GPIOG
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define LED_G1_Pin GPIO_PIN_1
#define LED_G1_GPIO_Port GPIOG
#define BEEP_Pin GPIO_PIN_6
#define BEEP_GPIO_Port GPIOH
#define OLED_BUTTON_Pin GPIO_PIN_6
#define OLED_BUTTON_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define OLED_MOSI_Pin GPIO_PIN_7
#define OLED_MOSI_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define SD_EXTI_Pin GPIO_PIN_15
#define SD_EXTI_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DJI_BOARD_TYPE_C_GENERAL
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
