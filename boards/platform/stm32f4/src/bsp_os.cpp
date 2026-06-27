/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "bsp_os.h"

#include "cmsis_os.h"
#include "task.h"

static TIM_HandleTypeDef* htim_os = nullptr;

/**
 * @brief override FreeRTOS weak function to configure the timer used for
 * generating run-time stats
 */
extern "C" void configureTimerForRunTimeStats(void) {
    if (!htim_os)
        return;
    __HAL_TIM_SET_AUTORELOAD(htim_os, 0xffffffff);
    __HAL_TIM_SET_COUNTER(htim_os, 0);
    __HAL_TIM_ENABLE(htim_os);
}

extern "C" unsigned long getRunTimeCounterValue(void) {
    if (!htim_os)
        return 0;
    return htim_os->Instance->CNT;
}

namespace bsp {

    void SetHighresClockTimer(TIM_HandleTypeDef* htim) {
        htim_os = htim;
        // 立即启动定时器
        // main.c 会先执行 MX_FREERTOS_Init()，此时并没有初始化 FreeRTOS 时基定时器。
        // FreeRTOS 定时器会在 main.c 的 osKernelStart() 中初始化时基定时器。
        // 但是 MotorCanBase.cpp 会执行断言 RM_ASSERT_TRUE(bsp::GetHighresTickMicroSec() != 0, "Highres timer not
        // initialized") 所以我们需要提前初始化一次高精度定时器以使 MotorCANBase 的断言能够通过。
        __HAL_TIM_SET_AUTORELOAD(htim_os, 0xffffffff);
        __HAL_TIM_SET_COUNTER(htim_os, 0);
        __HAL_TIM_ENABLE(htim_os);
    }

    uint16_t GetHighresTickMicroSec(void) {
        return getRunTimeCounterValue();
    }
    uint32_t GetHighresTickMilliSec(void) {
        return HAL_GetTick();
    }

} /* namespace bsp */
