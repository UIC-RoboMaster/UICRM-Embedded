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
#include "heater.h"

namespace driver {

    Heater::Heater(heater_init_t init) : pid_() {
        temp_ = init.target_temp;
        pwm_ = init.pwm;
        pwm_->Start();
        float* pid_param = init.pid_param;
        float heater_I_limit = init.heater_I_limit;
        float heater_output_limit = init.heater_output_limit;
        pid_.Reinit(pid_param, heater_I_limit, heater_output_limit);
    }

    float Heater::Update(float real_temp) {
        if (real_temp < temp_ - 1)
            pid_.cumulated_err_ = 0;
        float output = pid_.ComputeOutput(temp_ - real_temp);
        output = output > 0 ? output : 0;
        if (real_temp > temp_ + 0.5)
            output = 0;
        pwm_->SetPulseWidth((uint32_t)output);
        return output;
    }
}  // namespace driver
