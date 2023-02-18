#pragma once

#include "tim.h"

namespace bsp {

/**
 * @brief set the timer to be used to generate high resolution ticks in [us]
 *
 * @param htim  HAL timer handle pointer
 *
 * @note this has to be called before starting rtos scheduler
 */
void SetHighresClockTimer(TIM_HandleTypeDef* htim);

/**
 * @brief get the current counter value of the highres timer in [us]
 *
 * @return high res tick in [us] (0 if highres clock not set)
 */
uint32_t GetHighresTickMicroSec(void);

} /* namespace bsp */
