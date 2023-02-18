#include "bsp_os.h"

#include "cmsis_os.h"
#include "task.h"

static TIM_HandleTypeDef* htim_os = nullptr;

/**
 * @brief override FreeRTOS weak function to configure the timer used for generating run-time stats
 */
extern "C" void configureTimerForRunTimeStats(void) {
  if (!htim_os) return;
  __HAL_TIM_SET_AUTORELOAD(htim_os, 0xffffffff);
  __HAL_TIM_SET_COUNTER(htim_os, 0);
  __HAL_TIM_ENABLE(htim_os);
}

extern "C" unsigned long getRunTimeCounterValue(void) {
  if (!htim_os) return 0;
  return htim_os->Instance->CNT;
}

namespace bsp {

void SetHighresClockTimer(TIM_HandleTypeDef* htim) { htim_os = htim; }

uint32_t GetHighresTickMicroSec(void) { return getRunTimeCounterValue(); }

} /* namespace bsp */
