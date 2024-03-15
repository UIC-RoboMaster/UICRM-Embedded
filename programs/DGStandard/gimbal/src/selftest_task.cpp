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

#include "selftest_task.h"

osThreadId_t selftestTaskHandle;

selftest_t selftest;

void selftestTask(void* arg) {
    UNUSED(arg);
    uint8_t i = 0;
    while (true) {
        // Test Can Motor

        // 将连接状态设置为false，等待更新
        yaw_motor->connection_flag_ = false;
        pitch_motor->connection_flag_ = false;
        load_motor->connection_flag_ = false;
        // Test DBUS
        dbus->connection_flag_ = false;
        // Test Referee
        referee->connection_flag_ = false;
        if (i == 0)
            refereerc->connection_flag_ = false;
        osDelay(DETECT_OS_DELAY);

        // 获取最新连接状态
        selftest.yaw_motor = yaw_motor->connection_flag_;
        selftest.pitch_motor = pitch_motor->connection_flag_;
        selftest.steering_motor = load_motor->connection_flag_;
        selftest.dbus = dbus->connection_flag_;
        selftest.referee = referee->connection_flag_;
        // 图传串口的传输速率较慢，所以每三次检测一次
        if (i == 2)
            selftest.refereerc = refereerc->connection_flag_;

        osDelay(DETECT_OS_DELAY);
        i++;
        if (i == 3) {
            i = 0;
        }
    }
}

void init_selftest() {
}