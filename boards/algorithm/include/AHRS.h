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

#pragma once
#include "MahonyAHRS.h"
#include "main.h"
//clang-format off
#include "arm_math.h"
//clang-format on
namespace control {
    class AHRS {
      public:
        AHRS(bool is_mag);

        void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my,
                    float mz);

        void Update(float gx, float gy, float gz, float ax, float ay, float az);

        void Cailbrate();

        bool IsCailbrated();

        float INS_angle[3];// yaw pitch roll

      private:
        float q[4];
        float g_zerodrift[3] = {0};
        bool is_mag_;
        bool cailb_flag_;
        bool cailb_done_;
        uint16_t calib_cnt_ = 0;
        float accel_[3];
        float gyro_[3];
        float mag_[3];

        float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
        float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
        float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
        const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f,
                                     0.002329458745586203f};

        void CailbrateHandler(float gx, float gy, float gz, float ax, float ay, float az, float mx,
                              float my, float mz);

        void INSCalculate();
    };
}  // namespace control