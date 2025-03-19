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

#include <arm_math.h>
#include <stdint.h>

namespace control {
    /*
     * 新功率控制模型
     * 输入电机的总电流包括以下部分：
     * - 输出功率 = k1 * 角速度 * 转矩电流
     * - 摩擦损耗 = k2 * 角速度^2
     * - 电阻发热 = k3 * 转矩电流^2
     * - 静态电流 = k4
     */
    class NewPowerLimit {
      public:
        typedef struct {
            float k1, k2, k3, k4;
        } power_param_t;

      private:
        power_param_t params[4];

        /**
         * @brief 利用电动机转矩电流，正向推算电调输入电流
         * @param param 功率模型参数
         * @param angular_velocity 电机角速度
         * @param turn_current 电机转矩电流
         * @return 在电机转矩电流下，输入电调的总电流
         */
        int16_t PowerModel(power_param_t param, float angular_velocity, int16_t turn_current);

        /**
         * @brief 利用求根公式，根据目标的电调输入电流，反推电动机转矩电流
         * @param param 功率模型参数
         * @param angular_velocity 电机角速度
         * @param turn_current 电机转矩电流，用于判断取哪一个解
         * @param target_current 目标输入电流
         * @return 要让电调消耗目标的输入电流，需要发送给电调的转矩电流
         */
        int16_t ReversePowerModel(power_param_t param, float angular_velocity, int16_t turn_current,
                                  int16_t target_current);

      public:
        NewPowerLimit(power_param_t params[4]);

        void LimitPower(int16_t* turn_current, float* angular_velocity, uint16_t max_power);
    };
}  // namespace control