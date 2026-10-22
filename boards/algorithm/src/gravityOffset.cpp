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

#include "gravityOffset.h"

#include "arm_math.h"

GravityOffset::GravityOffset(const GravityConfig& config) {
    Configure(config);
}

void GravityOffset::Configure(const GravityConfig& config) {
    torque_gravity_ = 0.0f;
    max_torque_ = 0.0f;

    if (config.m_front < 0.0f || config.l_front < 0.0f || config.m_rear < 0.0f || config.l_rear < 0.0f ||
        config.max_torque < 0.0f) {
        return;
    }

    // 固定幅值 K = g * (m_front * l_front - m_rear * l_rear) [N·m]
    // 前后重力矩方向相反；这里只计算幅值，不包含随实际 Pitch 变化的 cos 项
    const float gain = 9.8f * (config.m_front * config.l_front - config.m_rear * config.l_rear);
    // torque_gravity_ = s * K，正指令抬头时 s = +1，否则 s = -1。
    torque_gravity_ = config.motor_direction ? gain : -gain;
    max_torque_ = config.max_torque;
}

float GravityOffset::Update(float pitch_rad, bool enabled) const {
    if (!enabled) {
        return 0.0f;
    }

    // 当前补偿 tau_ff = s * g * (m_front*l_front - m_rear*l_rear) * cos(pitch_rad)
    // pitch_rad 为实际仰角：水平 0 rad，抬头为正；输出为 Pitch 轴转矩 [N·m]
    const float torque = torque_gravity_ * arm_cos_f32(pitch_rad);
    if (torque > max_torque_) {
        return max_torque_;
    }
    if (torque < -max_torque_) {
        return -max_torque_;
    }
    return torque;
}
