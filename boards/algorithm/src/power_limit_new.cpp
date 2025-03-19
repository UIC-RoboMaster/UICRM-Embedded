/*###########################################################
 # Copyright (c) 2025. BNU-HKBU UIC RoboMaster              #
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

#include "power_limit_new.h"

// clang-format off
#include <main.h> // add __FPU_PRESENT
#include <arm_math.h>
#include <stdint.h>
// clang-format on

namespace control {
    NewPowerLimit::NewPowerLimit(power_param_t params[4]) {
        for (int i = 0; i < 4; i++) {
            this->params[i] = params[i];
        }
    }

    int16_t NewPowerLimit::PowerModel(power_param_t param, float angular_velocity,
                                      int16_t turn_current) {
        // I = k1ωτ + k2ω^2 + k3τ^2 + k4
        return param.k1 * angular_velocity * turn_current +
               param.k2 * angular_velocity * angular_velocity +
               param.k3 * turn_current * turn_current + param.k4;
    }

    int16_t NewPowerLimit::ReversePowerModel(power_param_t param, float angular_velocity,
                                             int16_t turn_current, int16_t target_current) {
        // (k3)τ^2 + (k1ω)τ + (k2ω^2 + k4 - I) = 0
        float a = param.k3;
        float b = param.k1 * angular_velocity;
        float c = param.k2 * angular_velocity * angular_velocity + param.k4 - target_current;
        float sqrt_delta = sqrt(b * b - 4 * a * c);
        float x1 = (-b + sqrt_delta) / (2 * a);
        float x2 = (-b - sqrt_delta) / (2 * a);
        if (abs(x1 - turn_current) < abs(x2 - turn_current)) {
            return x1;
        } else {
            return x2;
        }
    }

    void NewPowerLimit::LimitPower(int16_t* turn_current, float* angular_velocity,
                                   uint16_t max_power) {
        int16_t motor_input_current[4];
        int16_t forward_current = 0;
        int16_t reverse_current = 0;
        for (int i = 0; i < 4; i++) {
            motor_input_current[i] = PowerModel(params[i], angular_velocity[i], turn_current[i]);
            if (motor_input_current[i] > 0)
                forward_current += motor_input_current[i];
            else
                reverse_current += motor_input_current[i];
        }

        // 如果功率不超限，直接跳过
        if (forward_current + reverse_current < max_power)
            return;

        // 计算消耗功率的电机，应该将功率缩放到多少。
        float forward_ratio = 1.0 * (max_power - reverse_current) / forward_current;
        for (int i = 0; i < 4; i++) {
            if (motor_input_current[i] > 0)
                turn_current[i] = ReversePowerModel(params[i], angular_velocity[i], turn_current[i],
                                                    forward_ratio * motor_input_current[i]);
        }
    }
}  // namespace control