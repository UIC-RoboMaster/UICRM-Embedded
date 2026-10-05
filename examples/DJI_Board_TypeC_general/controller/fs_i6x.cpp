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
#include "controller/fs_i6x_receiver.h"

namespace {
    remote::FsI6xReceiver* receiver = nullptr;

    const char* PositionName(remote::FsI6xTwoPosition position) {
        return position == remote::FsI6xTwoPosition::Down ? "DOWN" : "UP";
    }

    const char* PositionName(remote::FsI6xThreePosition position) {
        switch (position) {
            case remote::FsI6xThreePosition::Down:
                return "DOWN";
            case remote::FsI6xThreePosition::Middle:
                return "MID";
            case remote::FsI6xThreePosition::Up:
                return "UP";
        }
        return "UNKNOWN";
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);

    receiver = new remote::FsI6xReceiver(&huart3);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    while (true) {
        remote::FsI6xFrame state;
        bool online;
        bool has_state;
        uint32_t sequence;
        uint32_t last_valid_ms;

        // USART3 优先级为 4，FreeRTOS 临界区不能屏蔽它。
        // 保存并短暂关闭中断，拷贝后恢复原状态；打印和延时在保护区外。
        const uint32_t interrupt_mask = __get_PRIMASK();
        __disable_irq();
        online = receiver->IsOnline();
        has_state = receiver->GetState(state);
        sequence = receiver->GetSequence();
        last_valid_ms = receiver->GetLastValidTimeMs();
        __set_PRIMASK(interrupt_mask);

        set_cursor(0, 0);
        clear_screen();
        print("FS-i6X / iA10B: %s\r\n", online ? "ONLINE" : "OFFLINE");
        print("Sequence: %lu\r\n", static_cast<unsigned long>(sequence));
        print("Last valid: %lu ms\r\n", static_cast<unsigned long>(last_valid_ms));
        if (has_state) {
            print("Frame lost: %d  Failsafe: %d  Flags: 0x%02X\r\n",
                  state.frame_lost, state.failsafe, static_cast<unsigned>(state.flags));
            print("Right X/Y: %d / %d\r\n", state.channels[0], state.channels[1]);
            print("Left X/Y: %d / %d\r\n", state.channels[3], state.channels[2]);
            print("VrA/VrB: %d / %d\r\n", state.channels[4], state.channels[5]);
            print("SwA: %s  SwB: %s  SwC: %s  SwD: %s\r\n",
                  PositionName(state.sw_a), PositionName(state.sw_b), PositionName(state.sw_c),
                  PositionName(state.sw_d));
            print("Usable: %d\r\n", online && !state.failsafe && !state.frame_lost);
            for (size_t i = 0; i < state.raw_channels.size(); ++i) {
                print("Raw CH%u: %u\r\n", static_cast<unsigned>(i + 1),
                      static_cast<unsigned>(state.raw_channels[i]));
            }
            // 离线仍可读到最后有效帧，在线也可能失控；实际控制须分别检查。
        } else {
            print("Waiting for a valid SBUS frame\r\n");
        }
        osDelay(100);
    }
}
