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

#include "DjiMotorBase.h"

namespace driver {

/**
 * @brief DJI GM6020 云台电机配置
 *
 * 编码器: 8192 线/圈，内置绝对位置
 * raw_current ∈ [-30000, 30000]
 */
struct Motor6020Config {
    static constexpr int16_t MAX_OUTPUT_CURRENT = 30000;
    static constexpr float CURRENT_TO_AMP = 3.0f / 16384.0f;
    static constexpr float ENCODER_RESOLUTION = 8192.0f;
    static constexpr float RATED_TORQUE_CONSTANT = 100.0f;  // mN·m/A
};

}  // namespace driver
