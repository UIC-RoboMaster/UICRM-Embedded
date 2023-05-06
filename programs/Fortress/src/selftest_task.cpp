/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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
    while (true) {
        // Test Can Motor
        fl_motor->connection_flag_ = false;
        fr_motor->connection_flag_ = false;
        bl_motor->connection_flag_ = false;
        br_motor->connection_flag_ = false;
        yaw_motor->connection_flag_ = false;
        pitch_motor->connection_flag_ = false;
        steering_motor->connection_flag_ = false;
        // Test DBUS
        dbus->connection_flag_ = false;
        osDelay(DETECT_OS_DELAY);
        selftest.fl_motor = fl_motor->connection_flag_;
        selftest.fr_motor = fr_motor->connection_flag_;
        selftest.bl_motor = bl_motor->connection_flag_;
        selftest.br_motor = br_motor->connection_flag_;
        selftest.yaw_motor = yaw_motor->connection_flag_;
        selftest.pitch_motor = pitch_motor->connection_flag_;
        selftest.steering_motor = steering_motor->connection_flag_;
        selftest.dbus = dbus->connection_flag_;
        selftest.imu_cali = imu->CaliDone();
        selftest.imu_temp = imu->Temp > 43.0f && imu->Temp < 50.0f;
        osDelay(DETECT_OS_DELAY);
    }
}

void init_selftest() {
}