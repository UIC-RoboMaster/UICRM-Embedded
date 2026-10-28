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
#include "dbus.h"

namespace {
    controller::DBUS* receiver = nullptr;
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);

    receiver = new controller::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    while (true) {
        controller::DbusFrame state;
        const bool has_state = receiver->GetState(state);
        const bool online = receiver->IsOnline();
        const uint32_t sequence = receiver->GetSequence();
        const uint32_t last_valid_ms = receiver->GetLastValidTimeMs();

        set_cursor(0, 0);
        clear_screen();
        print("DT7: %s\r\n", online ? "ONLINE" : "OFFLINE");
        print("Sequence: %lu\r\n", static_cast<unsigned long>(sequence));
        print("Last valid: %lu ms\r\n", static_cast<unsigned long>(last_valid_ms));
        if (has_state) {
            // 离线仍保留最后有效帧；实际控制必须先检查 online。
            print("Right X/Y: %d / %d\r\n", state.channels[0], state.channels[1]);
            print("Left X/Y: %d / %d\r\n", state.channels[2], state.channels[3]);
            print("Dial: %d\r\n", state.channels[4]);
            print(
                "Switch L/R: %s / %s\r\n",
                controller::SwitchName(state.left_switch),
                controller::SwitchName(state.right_switch)
            );
            print("Mouse X/Y: %d / %d\r\n", state.mouse.x, state.mouse.y);
            print("Mouse Z: %d\r\n", state.mouse.z);
            print("Mouse L/R: %d / %d\r\n", state.mouse.left, state.mouse.right);
            print("Keyboard: 0x%04X\r\n", static_cast<unsigned>(state.keyboard.code));
            const auto& key = state.keyboard.bit;
            print("W/S/A/D: %d %d %d %d\r\n", key.W, key.S, key.A, key.D);
            print("SHIFT/CTRL: %d %d\r\n", key.SHIFT, key.CTRL);
            print("Q/E/R/F/G: %d %d %d %d %d\r\n", key.Q, key.E, key.R, key.F, key.G);
            print("Z/X/C/V/B: %d %d %d %d %d\r\n", key.Z, key.X, key.C, key.V, key.B);
        } else {
            print("Waiting for a valid frame\r\n");
        }
        osDelay(100);
    }
}
