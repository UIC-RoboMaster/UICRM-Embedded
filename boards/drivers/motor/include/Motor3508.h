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
 * @brief DJI M3508/P19 减速电机配置
 *
 * C620 电调: raw_current ∈ [-16384, 16384]
 * 编码器: 8192 线/圈
 */
struct Motor3508Config {
    static constexpr int16_t MAX_OUTPUT_CURRENT = 12288;
    static constexpr float CURRENT_TO_AMP = 3.0f / 16384.0f;
    static constexpr float ENCODER_RESOLUTION = 8192.0f;
    static constexpr float RATED_TORQUE_CONSTANT = 250.0f;  // mN·m/A
    static constexpr float ORIGINAL_TRANSMISSION_RATIO = 3591.0f / 187.0f;
};

}  // namespace driver

