/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

/**
 * @file chassis_can_bridge.cpp
 * @brief 底盘 CAN 协议编码、接收校验及带超时检查的缓存读取实现
 */

#include "chassis_can_bridge.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace {
    /**
     * @brief 在 CanBridge 注册 CAN 回调前检查构造参数
     * @param can 待检查的硬件 CAN 对象
     * @param config 待检查的本机与对端地址配置
     * @return 通过检查的 CAN 指针。
     * @note CAN 指针为空或任一地址为 0 时触发断言。
     */
    bsp::CAN* CheckedCan(bsp::CAN* can, const communication::chassis_can_bridge_config& config) {
        // Validate before CanBridge's constructor registers with the CAN bus.
        RM_ASSERT_TRUE(can != nullptr, "Missing chassis CAN bus");
        RM_ASSERT_TRUE(config.local_address != 0, "Missing chassis local address");
        RM_ASSERT_TRUE(config.peer_address != 0, "Missing chassis peer address");
        return can;
    }

    /**
     * @brief 缓存读取期间的中断保护，退出作用域时恢复原有 PRIMASK
     * @note 调用前已屏蔽中断时，析构后仍保持屏蔽状态。
     */
    class InterruptGuard {
      public:
        /** @brief 保存当前中断屏蔽状态并关闭中断 */
        InterruptGuard() : primask_(__get_PRIMASK()) {
            __disable_irq();
        }
        /** @brief 恢复进入临界区前的中断屏蔽状态 */
        ~InterruptGuard() {
            __set_PRIMASK(primask_);
        }

      private:
        uint32_t primask_;  ///< 进入临界区前保存的 PRIMASK
    };
}  // namespace

namespace communication {
    // Float16 hardware conversion using ARM VCVTB instruction
    inline uint16_t chassis_can_bridge::Float32ToFloat16(float f) {
#if defined(__arm__) || defined(__thumb__)
        uint16_t result;
        __asm__ volatile("vcvtb.f16.f32 %0, %1" : "=t"(result) : "t"(f));
#else
        const _Float16 half = static_cast<_Float16>(f);
        uint16_t result;
        std::memcpy(&result, &half, sizeof(result));
#endif
        return result;
    }

    inline float chassis_can_bridge::Float16ToFloat32(uint16_t h) {
#if defined(__arm__) || defined(__thumb__)
        float result;
        __asm__ volatile("vcvtb.f32.f16 %0, %1" : "=t"(result) : "t"(h));
        return result;
#else
        _Float16 half;
        std::memcpy(&half, &h, sizeof(h));
        return static_cast<float>(half);
#endif
    }

    chassis_can_bridge::chassis_can_bridge(bsp::CAN* can, const chassis_can_bridge_config& config)
        : bridge_(CheckedCan(can, config), config.local_address), config_(config) {
        const chassis_register registers[] = {
            chassis_register::heartbeat_gimbal,
            chassis_register::heartbeat_chassis,
            chassis_register::command,
            chassis_register::feedback,
            chassis_register::power_limit,
        };
        for (auto reg : registers)
            bridge_.RegisterRxCallback(static_cast<uint8_t>(reg), ReceiveWrapper, this);
    }

    void chassis_can_bridge::Send(chassis_register reg, can_bridge_type_e type, can_bridge_data_t data) {
        can_bridge_ext_id_t id = {};
        id.data.rx_id = config_.peer_address;
        id.data.reg = static_cast<uint8_t>(reg);
        id.data.type = type;
        bridge_.Send(id, data);
    }

    void chassis_can_bridge::SendCommand(
        const control::chassis_speed& command, bool chassis_enable, bool system_active
    ) {
        const bool valid = std::isfinite(command.x) && std::isfinite(command.y) && std::isfinite(command.turn);
        if (!valid)
            return;

        can_bridge_data_t data = {};
        // bytes 0-1: vx_half
        uint16_t vx_half = Float32ToFloat16(command.x);
        data.data_eight_uint8.data[0] = vx_half & 0xFF;
        data.data_eight_uint8.data[1] = (vx_half >> 8) & 0xFF;
        // bytes 2-3: vy_half
        uint16_t vy_half = Float32ToFloat16(command.y);
        data.data_eight_uint8.data[2] = vy_half & 0xFF;
        data.data_eight_uint8.data[3] = (vy_half >> 8) & 0xFF;
        // bytes 4-5: vt_half
        uint16_t vt_half = Float32ToFloat16(command.turn);
        data.data_eight_uint8.data[4] = vt_half & 0xFF;
        data.data_eight_uint8.data[5] = (vt_half >> 8) & 0xFF;
        // byte 6: flags
        uint8_t flags = 0;
        if (chassis_enable)
            flags |= (1 << 0);
        if (system_active)
            flags |= (1 << 1);
        data.data_eight_uint8.data[6] = flags;
        // byte 7: reserved
        data.data_eight_uint8.data[7] = 0;

        Send(chassis_register::command, CAN_BRIDGE_TYPE_EIGHT_UINT8, data);
    }

    void chassis_can_bridge::SendPower(const control::chassis_power& power) {
        // Voltage 0 deliberately means use the chassis board's local voltage.
        if (!std::isfinite(power.max_watt) || power.max_watt < 0 || power.max_watt > 255 ||
            !std::isfinite(power.voltage) || power.voltage < 0 || power.buffer_percent > 100)
            return;
        can_bridge_data_t data = {};
        data.data_eight_uint8.data[0] = power.enabled ? 1 : 0;
        data.data_eight_uint8.data[1] = static_cast<uint8_t>(power.max_watt);
        data.data_eight_uint8.data[2] = power.buffer_percent;
        data.data_two_float.data[1] = power.voltage;
        Send(chassis_register::power_limit, CAN_BRIDGE_TYPE_TWO_FLOAT, data);
    }

    void chassis_can_bridge::SendFeedback(
        const control::chassis_state& state, uint8_t motors_online, uint8_t status_flags
    ) {
        if (!std::isfinite(state.feedback_vx) || !std::isfinite(state.feedback_vy) || !std::isfinite(state.feedback_vt))
            return;

        can_bridge_data_t data = {};
        // bytes 0-1: vx_half
        uint16_t vx_half = Float32ToFloat16(state.feedback_vx);
        data.data_eight_uint8.data[0] = vx_half & 0xFF;
        data.data_eight_uint8.data[1] = (vx_half >> 8) & 0xFF;
        // bytes 2-3: vy_half
        uint16_t vy_half = Float32ToFloat16(state.feedback_vy);
        data.data_eight_uint8.data[2] = vy_half & 0xFF;
        data.data_eight_uint8.data[3] = (vy_half >> 8) & 0xFF;
        // bytes 4-5: vt_half
        uint16_t vt_half = Float32ToFloat16(state.feedback_vt);
        data.data_eight_uint8.data[4] = vt_half & 0xFF;
        data.data_eight_uint8.data[5] = (vt_half >> 8) & 0xFF;
        // byte 6: motors_online
        data.data_eight_uint8.data[6] = motors_online;
        // byte 7: status_flags
        data.data_eight_uint8.data[7] = (status_flags & ~1u) | (state.enabled ? 1u : 0u);

        Send(chassis_register::feedback, CAN_BRIDGE_TYPE_EIGHT_UINT8, data);
    }

    void chassis_can_bridge::SendHeartbeat(bool is_gimbal, uint32_t uptime_ms) {
        can_bridge_data_t data = {};
        data.data_two_uint32.data[0] = tx_heartbeat_seq_++;
        data.data_two_uint32.data[1] = uptime_ms;

        chassis_register reg = is_gimbal ? chassis_register::heartbeat_gimbal : chassis_register::heartbeat_chassis;
        Send(reg, CAN_BRIDGE_TYPE_TWO_UINT32, data);
    }

    void chassis_can_bridge::ReceiveWrapper(can_bridge_ext_id_t id, can_bridge_data_t data, void* args) {
        static_cast<chassis_can_bridge*>(args)->Receive(id, data);
    }

    void chassis_can_bridge::Receive(can_bridge_ext_id_t id, can_bridge_data_t data) {
        if (id.data.tx_id != config_.peer_address)
            return;

        const auto reg = static_cast<chassis_register>(id.data.reg);
        const uint32_t now = HAL_GetTick();

        if (reg == chassis_register::command) {
            if (id.data.type != CAN_BRIDGE_TYPE_EIGHT_UINT8)
                return;
            // bytes 0-1: vx_half
            uint16_t vx_half = data.data_eight_uint8.data[0] | (data.data_eight_uint8.data[1] << 8);
            // bytes 2-3: vy_half
            uint16_t vy_half = data.data_eight_uint8.data[2] | (data.data_eight_uint8.data[3] << 8);
            // bytes 4-5: vt_half
            uint16_t vt_half = data.data_eight_uint8.data[4] | (data.data_eight_uint8.data[5] << 8);
            // byte 6: flags
            uint8_t flags = data.data_eight_uint8.data[6];
            // byte 7: reserved (unused)

            float vx = Float16ToFloat32(vx_half);
            float vy = Float16ToFloat32(vy_half);
            float vt = Float16ToFloat32(vt_half);

            if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vt))
                return;

            command_.x = vx;
            command_.y = vy;
            command_.turn = vt;
            command_chassis_enable_ = (flags & 0x01) != 0;
            command_system_active_ = (flags & 0x02) != 0;
            command_ms_ = now;
            command_seen_ = true;
            return;
        }

        if (reg == chassis_register::power_limit) {
            if (id.data.type != CAN_BRIDGE_TYPE_TWO_FLOAT || data.data_eight_uint8.data[0] > 1 ||
                data.data_eight_uint8.data[2] > 100 || !std::isfinite(data.data_two_float.data[1]) ||
                data.data_two_float.data[1] < 0)
                return;
            power_.enabled = data.data_eight_uint8.data[0] != 0;
            power_.max_watt = data.data_eight_uint8.data[1];
            power_.buffer_percent = data.data_eight_uint8.data[2];
            power_.voltage = data.data_two_float.data[1];
            power_ms_ = now;
            power_seen_ = true;
            return;
        }

        if (reg == chassis_register::feedback) {
            if (id.data.type != CAN_BRIDGE_TYPE_EIGHT_UINT8)
                return;
            // bytes 0-1: vx_half
            uint16_t vx_half = data.data_eight_uint8.data[0] | (data.data_eight_uint8.data[1] << 8);
            // bytes 2-3: vy_half
            uint16_t vy_half = data.data_eight_uint8.data[2] | (data.data_eight_uint8.data[3] << 8);
            // bytes 4-5: vt_half
            uint16_t vt_half = data.data_eight_uint8.data[4] | (data.data_eight_uint8.data[5] << 8);
            // byte 6: motors_online
            uint8_t motors_online = data.data_eight_uint8.data[6];
            // byte 7: status_flags
            uint8_t status_flags = data.data_eight_uint8.data[7];

            float vx = Float16ToFloat32(vx_half);
            float vy = Float16ToFloat32(vy_half);
            float vt = Float16ToFloat32(vt_half);

            if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vt))
                return;

            feedback_.feedback_vx = vx;
            feedback_.enabled = (status_flags & 1u) != 0;
            feedback_.feedback_vy = vy;
            feedback_.feedback_vt = vt;
            feedback_motors_online_ = motors_online;
            feedback_status_ = status_flags;
            feedback_ms_ = now;
            feedback_seen_ = true;
            return;
        }

        if (reg == chassis_register::heartbeat_gimbal || reg == chassis_register::heartbeat_chassis) {
            if (id.data.type != CAN_BRIDGE_TYPE_TWO_UINT32)
                return;
            uint32_t seq = data.data_two_uint32.data[0];
            uint32_t uptime_ms = data.data_two_uint32.data[1];

            if (reg == chassis_register::heartbeat_gimbal) {
                heartbeat_gimbal_seq_ = seq;
                heartbeat_gimbal_uptime_ms_ = uptime_ms;
                heartbeat_gimbal_ms_ = now;
                heartbeat_gimbal_seen_ = true;
            } else {
                heartbeat_chassis_seq_ = seq;
                heartbeat_chassis_uptime_ms_ = uptime_ms;
                heartbeat_chassis_ms_ = now;
                heartbeat_chassis_seen_ = true;
            }
            return;
        }
    }

    bool chassis_can_bridge::ReadCommand(
        control::chassis_speed& command, bool& chassis_enable, bool& system_active
    ) const {
        const InterruptGuard guard;
        command = {};
        chassis_enable = false;
        system_active = false;
        if (!command_seen_ || HAL_GetTick() - command_ms_ > config_.command_timeout_ms)
            return false;
        command = command_;
        chassis_enable = command_chassis_enable_;
        system_active = command_system_active_;
        return true;
    }

    bool chassis_can_bridge::ReadPower(control::chassis_power& power) const {
        const InterruptGuard guard;
        power = {};
        if (!power_seen_ || HAL_GetTick() - power_ms_ > config_.power_timeout_ms)
            return false;
        power = power_;
        return true;
    }

    bool chassis_can_bridge::ReadFeedback(
        control::chassis_state& state, uint8_t& motors_online, uint8_t& status_flags
    ) const {
        const InterruptGuard guard;
        state = {};
        motors_online = 0;
        status_flags = 0;
        if (!feedback_seen_ || HAL_GetTick() - feedback_ms_ > config_.feedback_timeout_ms)
            return false;
        state = feedback_;
        state.time = feedback_ms_;
        motors_online = feedback_motors_online_;
        status_flags = feedback_status_;
        return true;
    }

    bool chassis_can_bridge::ReadHeartbeat(bool from_gimbal, uint32_t& seq, uint32_t& uptime_ms) const {
        const InterruptGuard guard;
        seq = 0;
        uptime_ms = 0;

        if (from_gimbal) {
            if (!heartbeat_gimbal_seen_ || HAL_GetTick() - heartbeat_gimbal_ms_ > config_.heartbeat_timeout_ms)
                return false;
            seq = heartbeat_gimbal_seq_;
            uptime_ms = heartbeat_gimbal_uptime_ms_;
        } else {
            if (!heartbeat_chassis_seen_ || HAL_GetTick() - heartbeat_chassis_ms_ > config_.heartbeat_timeout_ms)
                return false;
            seq = heartbeat_chassis_seq_;
            uptime_ms = heartbeat_chassis_uptime_ms_;
        }
        return true;
    }
}  // namespace communication
