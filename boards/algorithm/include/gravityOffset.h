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

struct GravityConfig {
    float m_front = 0.0f;         // 前侧质量 [kg]
    float l_front = 0.0f;         // 转轴到前侧质心的距离 [m]
    float m_rear = 0.0f;          // 后侧质量 [kg]
    float l_rear = 0.0f;          // 转轴到后侧质心的距离 [m]
    bool motor_direction = true;  // 正指令抬头：true，否则：false
    float max_torque = 0.0f;   // 前馈转矩限幅 [N·m]，0 表示不输出
};

/**
 * @brief 直杆式 Pitch 云台的重力补偿。
 * @details 转轴位于前后负载之间，两侧重力矩方向相反，补偿取两侧力矩之差：
 *          tau_ff = s * g * (m_front * l_front - m_rear * l_rear) * cos(pitch_rad)。
 *          g = 9.8 m/s²；质量单位 kg，距离单位 m，角度单位 rad，转矩单位 N·m。
 *          s = +1（电机正指令抬头）或 -1（电机正指令低头）。
 *          Pitch 水平为 0，抬头为正；s = +1 时，前侧偏重产生正的抬头补偿。
 * @note 前后质心与转轴共线；Pitch 转轴近似水平。输出为 Pitch 轴转矩，
 *       接入电机前需按传动比及电机接口定义换算，不能直接与原始电流指令相加。
 */
class GravityOffset {
  public:
    explicit GravityOffset(const GravityConfig& config = GravityConfig{});

    /**
     * @brief 更新配置并预计算与 Pitch 角度无关的补偿幅值。
     * @details torque_gravity_ = s * 9.8 * (m_front * l_front - m_rear * l_rear)。
     *          此处尚未乘 cos(pitch_rad)，不是当前角度下的最终补偿。
     *          质量、距离和限幅为负时禁用输出。
     */
    void Configure(const GravityConfig& config);

    /**
     * @brief 根据实际 Pitch 角度计算最终补偿，每个控制周期调用。
     * @details tau_ff = torque_gravity_ * cos(pitch_rad)，
     *          返回值限制在 [-max_torque_, +max_torque_] 内。
     * @param pitch_rad 实际 Pitch 仰角 [rad]，水平为 0，抬头为正。
     * @param enabled 为 false 时返回 0；调用方仍需将 0 写入电机前馈接口。
     * @return 按电机方向修正并限幅的补偿转矩 [N·m]。
     */
    float Update(float pitch_rad, bool enabled = true) const;

  private:
    float torque_gravity_ = 0.0f;  // 带电机方向的固定幅值 [N·m]，尚未乘角度余弦
    float max_torque_ = 0.0f;
};
