/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

/*
 * 扫描 CAN1 与 CAN2 上的 AS5047P CAN 磁编模块，打印所有在线模块的读数
 * 模块发送 ID 为 0x400 + 模块 ID，模块 ID 存在模块自己的 EEPROM 里
 *
 * 用于标定：确定模块实际的发送 ID、转动方向，以及机械零位处的 raw_count
 *
 *
 * 开发板 C 型，不涉及电机，打印 RTT
 */

#include "AS5047PEncoder.h"
#include "MotorCanBase.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

namespace {
    constexpr uint16_t SCAN_BASE_ID = 0x400;
    constexpr uint16_t SCAN_COUNT = 16;

    driver::AS5047PEncoder* encoders_can1[SCAN_COUNT] = {nullptr};
    driver::AS5047PEncoder* encoders_can2[SCAN_COUNT] = {nullptr};
}  // namespace

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_rtt();
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);

    for (uint16_t i = 0; i < SCAN_COUNT; i++) {
        const uint16_t rx_id = (uint16_t)(SCAN_BASE_ID + i);
        encoders_can1[i] = new driver::AS5047PEncoder({
            .can = can1,
            .rx_id = rx_id,
            .offset = 0,
            .reversed = false,
        });
        encoders_can2[i] = new driver::AS5047PEncoder({
            .can = can2,
            .rx_id = rx_id,
            .offset = 0,
            .reversed = false,
        });
    }

    HAL_Delay(500);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        osDelay(100);

        print("\r\n[scan] std_id=0x%03X..0x%03X on can1 and can2\r\n", SCAN_BASE_ID,
              SCAN_BASE_ID + SCAN_COUNT - 1);

        uint16_t found = 0;
        for (uint16_t bus = 0; bus < 2; bus++) {
            driver::AS5047PEncoder** encoders = bus == 0 ? encoders_can1 : encoders_can2;
            for (uint16_t i = 0; i < SCAN_COUNT; i++) {
                if (!encoders[i]->IsOnline()) {
                    continue;
                }
                found++;
                print(
                    "can%u std_id=0x%03X raw_count=%u angle_deg=%.2f speed_rpm=%.1f "
                    "cumulated_turns=%+.3f\r\n",
                    bus + 1, SCAN_BASE_ID + i, encoders[i]->GetRawCount(),
                    encoders[i]->GetAngle() * 180.0f / PI, encoders[i]->GetRpm(),
                    (double)(encoders[i]->GetCumulatedAngle() / (2 * PI)));
            }
        }

        if (found == 0) {
            print("no encoder online\r\n");
        }
    }
}
