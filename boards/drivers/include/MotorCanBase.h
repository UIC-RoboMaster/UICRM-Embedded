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

#include "MotorBase.h"
#include "bsp_can.h"
#include "connection_driver.h"
#include "utils.h"

namespace driver {

/**
 * @brief CAN 通信电机的抽象基类
 *
 * 提供所有 CAN 总线电机共用的底层能力：
 * - 编码器角度/速度的存储与获取
 * - 传动比换算（输出轴角度/速度）
 * - 编码器回绕检测与多圈累计
 * - 连接心跳检测
 *
 * 本类不包含任何品牌特定的 PID 控制、CAN 分组传输、后台线程等能力。
 */
class MotorCANBase : public MotorBase, public ConnectionDriver {
  public:
    /**
     * @brief 基础构造函数
     * @param can              CAN 对象
     * @param rx_id            电机使用的 CAN 接收 ID
     * @param online_threshold 在线判定阈值（单位 ms），默认 30ms
     */
    MotorCANBase(bsp::CAN* can, uint16_t rx_id, uint32_t online_threshold = 30);

    /**
     * @brief 更新电机的反馈数据
     * @note 子类应在解析完 CAN 数据（设定 theta_ / omega_）后调用本方法
     * @param data[]  原始数据
     */
    virtual void UpdateData(const uint8_t data[]);

    /**
     * @brief 更新电机的保持状态（Holding）
     * @note 基类为空实现，由子类（如 DjiMotorBase）按需重写
     */
    virtual void UpdateHoldingState();

    /**
     * @brief 打印电机数据
     */
    virtual void PrintData() const = 0;

    /**
     * @brief 电机编码器角度，格式为 [rad]，范围为 [0, 2PI]
     */
    virtual float GetTheta() const;

    /**
     * @brief 电机编码器角速度，格式为 [rad / s]
     */
    virtual float GetOmega() const;

    /**
     * @brief 获得电机的累计输出轴角度（经过变速箱且编码器在变速箱之前），格式为 [rad]
     * @note 绝对模式下，认为输出轴只有一圈，输出轴角度不会累计，被限制在 [0, 2PI] 之间
     */
    virtual float GetOutputShaftTheta() const;

    /**
     * @brief 获得电机的输出轴角速度（经过变速箱且编码器在变速箱之前），格式为 [rad / s]
     */
    virtual float GetOutputShaftOmega() const;

    /**
     * @brief 获得电机的编码器角度与目标角度的角度差，格式为 [rad]
     * @param target 目标角度，格式为 [rad]
     * @return 与目标角度的弧度角度差，范围为 [-PI, PI]
     */
    virtual float GetThetaDelta(const float target) const;

    /**
     * @brief 获得电机的编码器角速度与目标角速度的角速度差，格式为 [rad / s]
     * @param target 目标角速度，格式为 [rad / s]
     * @return 与目标角速度的角速度差
     */
    virtual float GetOmegaDelta(const float target) const;

    /**
     * @return 电调反馈的转矩电流，单位为 [mA]
     */
    virtual int16_t GetCurr() const;

    /**
     * @return 电调反馈的温度，单位为 [℃]
     */
    virtual uint16_t GetTemp() const;

    /**
     * @brief 设置目标（默认空实现，由 DjiMotorBase 等有 PID 的子类重写）
     * @param target 设置输出轴的目标：角度 [RAD]、累计角度 [RAD]、角速度 [RAD/S]（取决于模式）
     * @param override 电机为角度控制模式下，还未达到之前的目标时，是否覆盖旧的目标
     */
    virtual void SetTarget(float target, bool override = true) {
        (void)target;
        (void)override;
    }

    /**
     * @brief 设置电机的减速箱比例
     * @param ratio 电机的减速箱比例
     */
    void SetTransmissionRatio(float ratio);

    /**
     * @brief 设置 ServoMotor 为 MotorCANBase 的友元
     */
    friend class ServoMotor;

    void Enable();
    void Disable();
    bool IsEnable() const;

    /**
     * @brief 设置绝对值模式
     * @param enable true 时输出轴角度不累计圈数，限制在 [0, 2PI] 之间
     */
    void SetAbsoluteMode(bool enable);

  protected:
    volatile float theta_;   // 编码器提供的角度值，单位为 [rad]
    volatile float omega_;   // 编码器提供的速度值，单位为 [rad/s]

    volatile float output_shaft_theta_;  // 电机输出轴的累计角度，单位为 [rad]
    volatile float output_shaft_omega_;  // 电机输出轴的速度，单位为 [rad/s]

    bool enable_;

    // angle control
    volatile float power_on_angle_ = 0;      /* 上电时的编码器角度，单位为 [rad] */
    volatile float relative_angle_ = 0;      /* 编码器相对于开机角度的角度，单位为 [rad] */
    volatile float cumulated_rad_ = 0;       /* 编码器累计圈数，按照 2*PI/ratio 加减 */
    volatile float output_cumulated_turns_ = 0;  /* 输出轴累计圈数，按照 2*PI 加减，单位为 [rad] */
    volatile float output_relative_angle_ = 0;   /* 输出轴在这一圈中的角度，单位为 [rad]，范围为 [0, 2PI] */

    FloatEdgeDetector* inner_wrap_detector_; /* detect motor motion across encoder boarder */
    FloatEdgeDetector* outer_wrap_detector_; /* detect motor motion across encoder boarder */

    float transmission_ratio_ = 1; /* 电机的减速比例 */

    bsp::CAN* can_;
    uint16_t rx_id_;

    // 上次运行 CalcOutput 时，最新收到的 CAN 数据包的时间戳（子类 CalcOutput 需要访问）
    uint32_t last_update_time_us_;

  private:
    bool absolute_mode_ = false;    // 绝对模式：输出轴不累计圈数
};

}  // namespace driver
