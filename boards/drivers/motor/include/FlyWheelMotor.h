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
#include "pid.h"

namespace driver {

/**
 * @brief 飞轮电机的初始化结构体
 */
typedef struct {
    DjiMotorBase* motor;    /* motor instance to be wrapped as a flywheel      */
    float max_speed;        /* desired turning speed of motor shaft, in [rad/s]  */
    float* omega_pid_param; /* pid parameter used to control speed of motor   */
    bool is_inverted;
} flywheel_t;

/**
 * @brief 飞轮电机的包装类
 *
 * @note 飞轮电机是一个特殊的类，使用 PID 控制器控制电机的角速度，使速度达到目标值且恒定
 * 通常用来控制摩擦轮的转速
 */
class FlyWheelMotor {
  public:
    /**
     * @brief 基础构造函数
     * @param data 飞轮电机的初始化结构体，参考 flywheel_t
     */
    FlyWheelMotor(flywheel_t data);
    /**
     * @brief 设置电机的旋转速度
     * @param speed 电机的目标旋转速度，单位为 [rad/s]
     */
    void SetSpeed(float speed);
    /**
     * @brief 计算电机的输出
     *
     * @note 这个函数不会直接控制电机，它只会计算电机的输出值
     */
    void CalcOutput();
    /**
     * @brief 获取电机的目标速度，单位为 [rad / s]
     *
     * @return 电机的目标速度
     */
    float GetTarget() const;
    /**
     * @brief 打印电机数据
     */
    void PrintData() const;

    /**
     * @brief 获取电机的角度，单位为 [rad]
     *
     * @return 电机的弧度角度，范围为 [0, 2PI]
     */
    float GetTheta() const;

    /**
     * @brief 获取电机的角度与目标角度的角度差，单位为 [rad]
     *
     * @param target 目标角度，单位为 [rad]
     *
     * @return 与目标角度的弧度角度差，范围为 [-PI, PI]
     */
    float GetThetaDelta(const float target) const;

    /**
     * @brief 获取电机的角速度，单位为 [rad / s]
     *
     * @return 电机的角速度
     */
    float GetOmega() const;

    /**
     * @brief 获取电机的角速度与目标角速度的角速度差，单位为 [rad / s]
     *
     * @param target 目标角速度，单位为 [rad / s]
     *
     * @return 与目标角速度的角速度差
     */
    float GetOmegaDelta(const float target) const;

    /**
     * @brief 更新电机的反馈数据
     *
     * @note 仅在 CAN 回调函数中使用，不要在其他地方调用
     *
     * @param data[]  原始数据
     */
    void UpdateData(const uint8_t data[]);

  private:
    DjiMotorBase* motor_;
    bool is_inverted_;
    float max_speed_;
    float target_speed_;
    control::PIDController omega_pid_;
};

}  // namespace driver
