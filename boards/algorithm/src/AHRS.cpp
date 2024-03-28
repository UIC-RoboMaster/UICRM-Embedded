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

#include "AHRS.h"

namespace control {

    AHRS::AHRS(bool is_mag) {
        is_mag_ = is_mag;
        cailb_flag_ = false;
        cailb_done_ = false;
        INS_angle[0] = 0;
        INS_angle[1] = 0;
        INS_angle[2] = 0;
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
        accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = accel_[0] = 0.0f;
        accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = accel_[1] = 0.0f;
        accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = accel_[2] = 0.0f;
    }
    void AHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx,
                      float my, float mz) {
        accel_[0] = ax;
        accel_[1] = ay;
        accel_[2] = az;
        gyro_[0] = gx;
        gyro_[1] = gy;
        gyro_[2] = gz;
        mag_[0] = mx;
        mag_[1] = my;
        mag_[2] = mz;
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];
        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] +
                            accel_[0] * fliter_num[2];
        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];
        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] +
                            accel_[1] * fliter_num[2];
        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];
        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] +
                            accel_[2] * fliter_num[2];
        if (cailb_done_) {
            MahonyAHRSupdate(q, gx - g_zerodrift[0], gy - g_zerodrift[1], gz - g_zerodrift[2],
                             accel_fliter_1[0], accel_fliter_1[1], accel_fliter_1[2], mx, my, mz);
            INSCalculate();
        } else if (cailb_flag_) {
            CailbrateHandler(gx, gy, gz, ax, ay, az, mx, my, mz);
        }
    }
    void AHRS::Update(float gx, float gy, float gz, float ax, float ay, float az) {
        accel_[0] = ax;
        accel_[1] = ay;
        accel_[2] = az;
        gyro_[0] = gx;
        gyro_[1] = gy;
        gyro_[2] = gz;
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];
        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] +
                            accel_[0] * fliter_num[2];
        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];
        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] +
                            accel_[1] * fliter_num[2];
        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];
        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] +
                            accel_[2] * fliter_num[2];
        if (cailb_done_) {
            MahonyAHRSupdateIMU(q, gx - g_zerodrift[0], gy - g_zerodrift[1], gz - g_zerodrift[2],
                                accel_fliter_1[0], accel_fliter_1[1], accel_fliter_1[2]);
            INSCalculate();
        } else if (cailb_flag_) {
            CailbrateHandler(gx, gy, gz, ax, ay, az, 0, 0, 0);
        }
    }
    void AHRS::CailbrateHandler(float gx, float gy, float gz, float ax, float ay, float az,
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
    void AHRS::Cailbrate() {
        cailb_flag_ = true;
        cailb_done_ = false;
    }

    bool AHRS::IsCailbrated() {
        return cailb_done_;
    }
    void AHRS::INSCalculate() {
        INS_angle[0] =
            atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
        INS_angle[1] = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
        INS_angle[2] =
            atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
    }
}  // namespace control