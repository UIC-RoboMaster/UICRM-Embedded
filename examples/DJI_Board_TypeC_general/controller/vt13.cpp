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

#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "task.h"

namespace {
    communication::Referee* link = nullptr;
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);

    // 图传模块发送端 UART：921600 baud、8 位数据、无校验、1 位停止位。
    huart6.Init.BaudRate = 921600;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.StopBits = UART_STOPBITS_1;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }

    auto* uart = new bsp::UART(&huart6);
    // Referee 内部注册 UART 回调和接收线程，由 Protocol 完成 A5/A9 分帧。
    link = new communication::Referee(uart);
    uart->SetupRx(300);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    while (true) {
        remote::Vt13Frame state;
        bool online;
        bool has_state;
        uint32_t sequence;
        uint32_t last_valid_ms;

        // 防止接收线程在拷贝状态期间切入；临界区内不打印、不延时。
        taskENTER_CRITICAL();
        const auto& receiver = link->Vt13();
        online = receiver.IsOnline(bsp::GetHighresTickMilliSec());
        has_state = receiver.GetState(state);
        sequence = receiver.GetSequence();
        last_valid_ms = receiver.GetLastValidTimeMs();
        taskEXIT_CRITICAL();

        set_cursor(0, 0);
        clear_screen();
        // 使用遥控有效帧的在线状态，不能用 Referee::IsOnline 代替。
        print("VT13: %s\r\n", online ? "ONLINE" : "OFFLINE");
        print("Sequence: %lu\r\n", static_cast<unsigned long>(sequence));
        print("Last valid: %lu ms\r\n", static_cast<unsigned long>(last_valid_ms));
        if (has_state) {
            // VT13 通道为 -660～660，中心 0，包含 ±10 死区；左摇杆的顺序是 Y/X。
            print("Right X/Y: %d / %d\r\n", state.channels[0], state.channels[1]);
            print("Left Y/X: %d / %d\r\n", state.channels[2], state.channels[3]);
            print("Dial: %d\r\n", state.channels[4]);
            const char* mode = state.mode == remote::Vt13Mode::C ? "C" :
                               state.mode == remote::Vt13Mode::N ? "N" : "S";
            print("Mode: %s  Pause: %d\r\n", mode, state.pause);
            print("Button L/R: %d / %d\r\n", state.left_button, state.right_button);
            print("Trigger: %d\r\n", state.trigger);
            print("Mouse X/Y: %d / %d\r\n", state.mouse.x, state.mouse.y);
            print("Mouse Z: %d\r\n", state.mouse.z);
            print("Mouse L/R/M: %d %d %d\r\n", state.mouse.left, state.mouse.right, state.mouse.middle);
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
