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

#include "QEKF.h"

namespace control {

    QEKF::QEKF(qekf_gettickoffset_t gettickdelta) : gettickdelta_(gettickdelta) {
        cailb_flag_ = false;
        cailb_done_ = false;
        INS_angle[0] = 0;
        INS_angle[1] = 0;
        INS_angle[2] = 0;
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;

        IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    }

    void QEKF::Update(float gx, float gy, float gz, float ax, float ay, float az) {
        ticks_count_current_ = gettickdelta_(&last_tick_);
        ticks_count_ += ticks_count_current_;

        accel_[0] = ax;
        accel_[1] = ay;
        accel_[2] = az;
        gyro_[0] = gx;
        gyro_[1] = gy;
        gyro_[2] = gz;

        if (cailb_done_) {
            IMU_QuaternionEKF_Update(gx - g_zerodrift[0], gy - g_zerodrift[1], gz - g_zerodrift[2],
                                     ax, ay, az, ticks_count_current_);
            INS_angle[0] = QEKF_INS.Yaw;
            INS_angle[1] = QEKF_INS.Pitch;
            INS_angle[2] = QEKF_INS.Roll;
        } else if (cailb_flag_) {
            CailbrateHandler(gx, gy, gz, ax, ay, az, 0, 0, 0);
        }
    }
    void QEKF::CailbrateHandler(float gx, float gy, float gz, float ax, float ay, float az,
                                float mx, float my, float mz) {
        UNUSED(ax);
        UNUSED(ay);
        UNUSED(az);
        UNUSED(mx);
        UNUSED(my);
        UNUSED(mz);
        if (calib_cnt_ < 2000) {
            calib_cnt_++;
            g_zerodrift[0] += gx;
            g_zerodrift[1] += gy;
            g_zerodrift[2] += gz;
        } else {
            g_zerodrift[0] /= 2000;
            g_zerodrift[1] /= 2000;
            g_zerodrift[2] /= 2000;
            cailb_done_ = true;
            cailb_flag_ = false;
        }
    }
    void QEKF::Cailbrate() {
        cailb_flag_ = true;
        cailb_done_ = false;
    }

    bool QEKF::IsCailbrated() {
        return cailb_done_;
    }
    void QEKF::INSCalculate() {
        INS_angle[0] =
            atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
        INS_angle[1] = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
        INS_angle[2] =
            atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
    }
}  // namespace control