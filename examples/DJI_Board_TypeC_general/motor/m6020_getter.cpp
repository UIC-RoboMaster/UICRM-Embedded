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

#include "DjiMotorBase.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

static bsp::CAN* can = nullptr;
static driver::Motor6020* motor1 = nullptr;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);

    // print_use_uart(&huart6, true, 921600);
    print_use_rtt();

    can = new bsp::CAN(&hcan2, true);
    motor1 = new driver::Motor6020(can, 0x206);
    // Disable output while keeping CAN angle feedback active for calibration.
    motor1->Disable();
    motor1->SetTransmissionRatio(1);
    motor1->SetMode(driver::DjiMotorBase::ABSOLUTE);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("GM6020 center calibration (CAN1, RX 0x209)\r\n");
        print("Output disabled. Move the mechanism to its mechanical center.\r\n");
        if (motor1->IsOnline()) {
            // Read the absolute encoder angle, independent of the power-on position.
            const float theta = motor1->GetTheta();
            print("Encoder angle: %.6f rad / %.3f deg\r\n", theta, theta * 180.0f / PI);
            print("At mechanical center, record: center_offset = %.6ff;\r\n", theta);
            // In the application: centered_theta = wrap(theta - center_offset, -PI, PI).
            // With THETA | OMEGA | ABSOLUTE and ratio 1, the center target is center_offset.
            print("Relative angle = wrap(theta - center_offset, -PI, PI)\r\n");
        } else {
            print("Motor offline: waiting for CAN feedback.\r\n");
        }
        osDelay(100);
    }
}
