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
#include "bsp_pwm.h"
#include "main.h"
#include "pid.h"

namespace driver {

    typedef struct {
        bsp::PWM* pwm;
        float target_temp;
        float* pid_param = new float[3]{160, 0.1, 0};
        float heater_I_limit = 800;
        float heater_output_limit = 500;
    } heater_init_t;

    class Heater {
      public:
        Heater(heater_init_t init);
        float Update(float real_temp);

      private:
        bsp::PWM* pwm_;
        float temp_;
        control::ConstrainedPID pid_;
    };

}  // namespace driver
