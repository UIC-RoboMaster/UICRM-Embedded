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

        void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

        void Update(float gx, float gy, float gz, float ax, float ay, float az);

        void Cailbrate();

        bool IsCailbrated();

        float INS_Angle[3];
      private:
        float q[4];
        float g_zerodrift[3]={0};
        bool is_mag_;
        bool cailb_flag_;
        bool cailb_done_;
        uint16_t calib_cnt_=0;

        void CailbrateHandler(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

        void INSCalculate();
    };
}