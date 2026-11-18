/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim12;

extern TIM_HandleTypeDef htim23;

extern TIM_HandleTypeDef htim24;

/* USER CODE BEGIN Private defines */
// 板级定时器分配：TIMxCLK = 270 MHz，下表定时器 PSC 均已分频到 1 MHz 计数
#define BOARD_TIM_PWM1 htim1          // PE9(CH1) / PE13(CH3)
#define BOARD_TIM_PWM2 htim2          // PA0(CH1) / PA2(CH3)

#define BOARD_TIM_IMUHEATER htim3
#define BOARD_TIM_IMUHEATER_CH 4      // PB1

#define BOARD_TIM_SYS htim24          // 1 MHz 32 位，高分辨率计时/运行统计

#define BOARD_TIM_BUZZER htim12
#define BOARD_TIM_BUZZER_CH 2         // PB15
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM12_Init(void);
void MX_TIM23_Init(void);
void MX_TIM24_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

