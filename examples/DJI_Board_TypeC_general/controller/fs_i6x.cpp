/*###########################################################
# Copyright (c) 2026-2027. BNBU/UIC RoboMaster             #
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

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "sbus.h"

static controller::I6x* receiver = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);

    // I6x 继承 SBUS 的 UART 接收，无需另外创建 SBUS 或注册回调。
    receiver = new controller::I6x(&huart3);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    while (true) {
        controller::I6xState state;
        controller::SbusFrame raw;

        const bool has_state = receiver->GetState(state) && receiver->GetRawState(raw);
        const bool online = receiver->IsOnline();
        const uint32_t sequence = receiver->GetSequence();
        const uint32_t last_valid_ms = receiver->GetLastValidTimeMs();

        set_cursor(0, 0);
        clear_screen();
        print("FS-i6X / iA10B: %s\r\n", online ? "ONLINE" : "OFFLINE");
        print("Sequence: %lu\r\n", static_cast<unsigned long>(sequence));
        print("Last valid: %lu ms\r\n", static_cast<unsigned long>(last_valid_ms));
        if (has_state) {
            print(
                "Frame lost: %d  Failsafe: %d  Flags: 0x%02X\r\n",
                state.frame_lost,
                state.failsafe,
                static_cast<unsigned>(raw.flags)
            );
            // CH1/2：右 X/Y；CH3/4：左 Y/X；CH5/6：VrA/VrB；CH7～10：SwA～SwD。
            // I6x 已统一映射到 ±660 并处理原始 ±10 死区；原始通道保留用于诊断。
            print("Right X/Y: %d / %d\r\n", state.channels[0], state.channels[1]);
            print("Left X/Y: %d / %d\r\n", state.channels[3], state.channels[2]);
            print("VrA/VrB: %d / %d\r\n", state.channels[4], state.channels[5]);
            print(
                "Switch A/B/C/D: %s / %s / %s / %s\r\n",
                controller::SwitchName(state.sw_a),
                controller::SwitchName(state.sw_b),
                controller::SwitchName(state.sw_c),
                controller::SwitchName(state.sw_d)
            );
            print("Usable: %d\r\n", online && !state.failsafe && !state.frame_lost);
            for (size_t i = 0; i < raw.channels.size(); ++i) {
                print("Raw CH%u: %u\r\n", static_cast<unsigned>(i + 1), static_cast<unsigned>(raw.channels[i]));
            }
            print("Digital CH17/18: %d / %d\r\n", raw.channel17, raw.channel18);
            // 离线仍可读到最后有效帧，在线也可能失控；实际控制须分别检查。
        } else {
            print("Waiting for a valid SBUS frame\r\n");
        }
        osDelay(100);
    }
}
