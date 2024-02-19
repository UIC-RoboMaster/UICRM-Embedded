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
#include "heater.h"

namespace driver {

    Heater::Heater(heater_init_t init) {
        control::ConstrainedPID::PID_Init_t pid_init={
            .kp = init.pid_param[0],
            .ki = init.pid_param[1],
            .kd = init.pid_param[2],
            .max_out = init.heater_output_limit,
            .max_iout = init.heater_I_limit,
            .deadband = 0, //死区
            .A = 0, //变速积分所能达到的最大值为A+B
            .B = 0, //启动变速积分的死区
            .output_filtering_coefficient = 0.1, //输出滤波系数
            .derivative_filtering_coefficient = 0, //微分滤波系数
            .mode = control::ConstrainedPID::Integral_Limit|control::ConstrainedPID::OutputFilter|control::ConstrainedPID::Trapezoid_Intergral,
        };
        pid_ = control::ConstrainedPID(pid_init);
        temp_ = init.target_temp;
        pwm_ = init.pwm;
        pwm_->Start();
        float* pid_param = init.pid_param;
        float heater_I_limit = init.heater_I_limit;
        float heater_output_limit = init.heater_output_limit;
        pid_.Reinit(pid_param, heater_I_limit, heater_output_limit);
    }

    float Heater::Update(float real_temp) {
        float output = pid_.ComputeOutput(temp_,real_temp);
        output = output > 0 ? output : 0;
        if (real_temp > temp_ + 0.5)
            output = 0;
        pwm_->SetPulseWidth((uint32_t)output);
        return output;
    }
}  // namespace driver
