// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2025/3/22.
//

#include "bsp_print.h"
#include "buzzer_task.h"
#include "gimbal_data.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "public_port.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "user_define.h"

void RM_RTOS_Init() {
    print_use_uart(&Debug_UART, true, 921600);
    init_can();
    init_remote();
    init_gimbal();
    init_shoot();
    init_buzzer();
    init_imu();
}

void RM_RTOS_Threads_Init(void) {
    // 分别启动每个任务
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
    print("RM_RTOS_Threads_Init");
}

void RM_Main_remote_mode() {
    char s[20];
    switch (remote_mode) {
        case REMOTE_MODE_STOP:
            strcpy(s, "STOP");
            break;
        case REMOTE_MODE_KILL:
            strcpy(s, "KILL");
            break;
        case REMOTE_MODE_FOLLOW:
            strcpy(s, "FOLLOW");
            break;
        case REMOTE_MODE_ADVANCED:
            strcpy(s, "ADVANCED");
            break;
        case REMOTE_MODE_PREPARE_HAND_MOVEMENT:
            strcpy(s, "STICK_SHIFT");
            break;

        default:
            strcpy(s, "UNKNOWN");
            break;
    }
    print("\n");
    print("Mode:%s\r\n", s);
}

void RM_Main_data(bool newline = false) {
    set_cursor(0, 0);
    clear_screen();
    print(
        "DBUS [CH0: %-4d] [CH1: %-4d] [CH2: %-4d] [CH3: %-4d] [TWL: %d] [SWL: %d] [SWR: %d]"
        "@ %d "
        "ms\r\n",
        dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr, dbus->ch4,
        dbus->timestamp);

    // print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
    //       ahrs->INS_angle[2]);
    if (newline) {
        print("\r\n");
    }
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(3000);
    Buzzer_Sing(DJI);
    protect_wraning_flag = false;
    while (true) {
        RM_Main_remote_mode();
        shoot_remote_mode();
        RM_Main_data(true);
        debug_gimbal(true);
        IMU_print();
        osDelay(50);
    }
}