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

#include "bsp_can.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "connection_driver.h"
#include "utils.h"
#include "arm_math.h"

namespace driver {

/**
 * @brief CAN 通信电机的抽象基类
 *
 * 提供所有 CAN 总线电机共用的能力：
 * - 编码器角度/速度的获取（虚函数，组件多态契约）
 * - 传动比换算（输出轴角度/速度）
 * - 编码器回绕检测与多圈累计
 * - 连接心跳检测
 *
 * 子类需：
 *  1. 提供 state_ 成员（包含 theta, omega, output_shaft_theta, enable, can 等字段）
 *  2. 实现 CanMotorBase 虚函数契约
 */
class CanMotorBase : public ConnectionDriver {
  public:
    virtual ~CanMotorBase() = default;

    // ── 组件契约 ──

    /**
     * @brief 设置目标
     * @param target 目标值：角度 [rad]、累计角度 [rad] 或角速度 [rad/s]（取决于子类模式）
     * @param override 角度控制模式下，未达旧目标时是否覆盖
     */
    virtual void SetTarget(float target, bool override = true) = 0;

    /**
     * @brief 获得电机转子角度
     * @return 编码器角度 [rad]
     */
    virtual float GetTheta() const = 0;

    /**
     * @brief 获得输出轴累计角度
     * @return 输出轴角度 [rad]
     */
    virtual float GetOutputShaftTheta() const = 0;

    /**
     * @brief 获得输出轴角速度
     * @return 输出轴角速度 [rad/s]
     */
    virtual float GetOutputShaftOmega() const = 0;

    /**
     * @brief 获得电机转子角速度
     * @return 角速度 [rad/s]
     */
    virtual float GetOmega() const = 0;

    /**
     * @brief 使能电机输出
     */
    virtual void Enable() = 0;

    /**
     * @brief 禁用电机输出
     */
    virtual void Disable() = 0;

    /**
     * @brief 查询电机是否使能
     * @return true 表示已使能
     */
    virtual bool IsEnable() const = 0;

    /**
     * @brief 获取当前输出电流指令
     * @return 输出值 [raw]
     */
    virtual int16_t GetOutput() = 0;

    /**
     * @brief 设置输出电流指令
     * @param val 输出值 [raw]
     */
    virtual void SetOutput(int16_t val) = 0;

    /**
     * @brief 更新电机的反馈数据
     * @param data 原始 CAN 数据
     */
    virtual void UpdateData(const uint8_t data[]) = 0;

    /**
     * @brief 打印电机调试数据
     */
    virtual void PrintData() const = 0;


    /**
     * @brief 获得转子角度与目标角度的差值
     * @param target 目标角度 [rad]
     * @return 角度差 [rad]
     */
    float GetThetaDelta(float target) const {
        return wrap<float>(target - GetTheta(), -PI, PI);
    }

    /**
     * @brief 获得转子角速度与目标角速度的差值
     * @param target 目标角速度 [rad/s]
     * @return 角速度差 [rad/s]
     */
    float GetOmegaDelta(float target) const {
        return target - GetOmega();
    }

  protected:
    /**
     * @brief 构造 CAN 电机基类
     * @param online_threshold 连接离线判定阈值（单位：ms）
     * @note 大多电机反馈频率默认为 1000 Hz，即 1 ms/帧，默认若连续 30 帧未收到反馈，则判定为离线
     */
    explicit CanMotorBase(uint32_t online_threshold = 30);

    /**
     * @brief 向 CAN 总线发送一帧标准数据
     * @param can   CAN 硬件对象
     * @param tx_id 发送报文标识符
     * @param data  数据缓冲区（至少 dlc 字节有效）
     * @param dlc   数据长度 [字节]，默认 8
     */
    static void TransmitFrame(bsp::CAN* can, uint16_t tx_id, const uint8_t data[8], uint8_t dlc = 8);

    /// CAN 接收回调函数指针类型（不经 vtable）
    using CanRxHandler = void (*)(void* ctx, const uint8_t data[]);

    /**
     * @brief 注册 CAN 接收回调
     * @param can     CAN 硬件对象
     * @param rx_id   电机反馈报文标识符
     * @param handler 接收回调函数指针
     * @param ctx     传入 handler 的上下文指针
     * @note 子类构造函数中调用一次即可，自动绑定到所属 CAN 的 rx_id
     */
    void RegisterCanCallback(bsp::CAN* can, uint16_t rx_id, CanRxHandler handler, void* ctx);

  private:
    /**
     * @brief BSP CAN 接收回调，转发到 CanRxHandler
     * @param data 原始 CAN 数据
     * @param args 指向 CanMotorBase 实例的指针
     */
    static void BspRxThunk(const uint8_t data[], void* args);

    CanRxHandler rx_handler_ = nullptr;
    void* rx_ctx_ = nullptr;
};

}  // namespace driver
