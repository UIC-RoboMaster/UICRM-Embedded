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

#include "DmMotorBase.h"

#include "arm_math.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "utils.h"

namespace driver {

// ===== Static member definitions =====

DmMotorBase* DmMotorBase::instances_[16] = {};
uint8_t DmMotorBase::instance_count_ = 0;
bool DmMotorBase::dm_thread_started_ = false;
bsp::Thread* DmMotorBase::dm_thread_ = nullptr;
uint32_t DmMotorBase::dm_output_period_us_ = 1000;

// ===== DmMotorBase =====

DmMotorBase::DmMotorBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
    : MotorCANBase<DmMotorBase>(50) {
    state_.can = can;
    state_.rx_id = rx_id;
    state_.tx_id = tx_id;
    // DM 电机使用绝对值编码器，无需等待上电角度
    state_.power_on_angle = 0;

    // 自注册到全局实例列表
    RM_ASSERT_LT(instance_count_, 16, "Too many DM motor instances");
    instances_[instance_count_++] = this;

    // 首次构造时启动后台线程
    if (!dm_thread_started_) {
        dm_thread_started_ = true;
        bsp::thread_init_t thread_init = {
            .func = DmMotorThread, .args = nullptr,
            .attr = {.name = "DmMotorTask",
                     .attr_bits = osThreadDetached,
                     .cb_mem = nullptr,
                     .cb_size = 0,
                     .stack_mem = nullptr,
                     .stack_size = 256 * 4,
                     .priority = (osPriority_t)osPriorityHigh,
                     .tz_module = 0,
                     .reserved = 0}};
        dm_thread_ = new bsp::Thread(thread_init);
        dm_thread_->Start();
    }
}

void DmMotorBase::MotorEnable() {
    uint8_t data[8] = {0};
    data[0] = 0xff;
    data[1] = 0xff;
    data[2] = 0xff;
    data[3] = 0xff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;
    data[7] = 0xfc;
    SendPacket(data);
}

void DmMotorBase::MotorDisable() {
    uint8_t data[8] = {0};
    data[0] = 0xff;
    data[1] = 0xff;
    data[2] = 0xff;
    data[3] = 0xff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;
    data[7] = 0xfd;
    SendPacket(data);
}

void DmMotorBase::SetZeroPos() {
    uint8_t data[8] = {0};
    data[0] = 0xff;
    data[1] = 0xff;
    data[2] = 0xff;
    data[3] = 0xff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;
    data[7] = 0xfe;
    SendPacket(data);
}

void DmMotorBase::SetOutputFrequency(uint32_t freq) {
    RM_ASSERT_GT(freq, 0, "Frequency must be positive");
    dm_output_period_us_ = 1000000 / freq;
}

void DmMotorBase::DmMotorThread(void* args) {
    (void)args;
    while (1) {
        for (uint8_t i = 0; i < instance_count_; i++) {
            instances_[i]->CalcOutput();
            // 分摊发送：bxCAN 只有 3 个 mailbox，连续发送 ≥4 帧会丢帧。
            // 每发完一个电机让出 CPU（~1ms），等待 CAN 硬件释放 mailbox。
            // FDCAN（H7）有硬件 TX FIFO，此延迟实际无害。
            if (instance_count_ >= 4) {
                osDelay(1);
            }
        }
        osDelay(dm_output_period_us_ / 1000);
    }
}

void DmMotorBase::CalcOutput() {
    if (!state_.enable) {
        return;
    }
    TransmitOutput();
}

void DmMotorBase::SetOutput(float position, float velocity, float kp, float kd, float torque) {
    state_.kp_setpoint = kp;
    state_.kd_setpoint = kd;
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
    state_.torque_feedforward = torque;
}

void DmMotorBase::SetOutput(float position, float velocity) {
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
}

void DmMotorBase::SetOutput(float velocity) {
    state_.velocity_setpoint = velocity;
}

void DmMotorBase::SetTarget(float target, bool override) {
    (void)override;
    RM_ASSERT_TRUE(state_.mode == VEL, "SetTarget(float) only valid in VEL mode");
    state_.velocity_setpoint = target;
}

void DmMotorBase::SetTarget(float position, float velocity, float kp, float kd, float t_ff) {
    RM_ASSERT_TRUE(state_.mode == MIT, "SetTarget(5 args) only valid in MIT mode");
    state_.kp_setpoint = kp;
    state_.kd_setpoint = kd;
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
    state_.torque_feedforward = t_ff;
}

void DmMotorBase::SetTarget(float position, float velocity) {
    RM_ASSERT_TRUE(state_.mode == POS_VEL, "SetTarget(2 args) only valid in POS_VEL mode");
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
}

float DmMotorBase::GetTorque() const {
    return state_.torque;
}

uint16_t DmMotorBase::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float DmMotorBase::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ===== DMMotor4310 =====

DMMotor4310::DMMotor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_m4310_mode_t mode)
    : DmMotorBase(can, rx_id, tx_id) {
    RegisterCanCallback();
    state_.mode = mode;
    /* following the CAN id format from the m4310 V2.1 document */
    if (mode == MIT) {
        state_.tx_id_actual = tx_id;
    } else if (mode == POS_VEL) {
        state_.tx_id_actual = tx_id + 0x100;
    } else if (mode == VEL) {
        state_.tx_id_actual = tx_id + 0x200;
    } else {
        RM_EXPECT_TRUE(false, "Invalid mode number!");
    }
}

void DMMotor4310::UpdateData(const uint8_t data[]) {
    state_.error = data[0] >> 4;
    state_.id = data[0] & 0x0f;
    state_.raw_position = data[1] << 8 | data[2];
    state_.raw_velocity = data[3] << 4 | (data[4] & 0xf0) >> 4;
    state_.raw_torque = data[5] | (data[4] & 0x0f) << 8;
    state_.mos_temperature = data[6];
    state_.rotor_temperature = data[7];

    state_.theta = DmMotorBase::uint_to_float(state_.raw_position, P_MIN, P_MAX, 16);
    state_.omega = DmMotorBase::uint_to_float(state_.raw_velocity, V_MIN, V_MAX, 12);
    state_.torque = DmMotorBase::uint_to_float(state_.raw_torque, T_MIN, T_MAX, 12);

    // 调基类做角度追踪
    ProcessAngleTracking();
}

void DMMotor4310::TransmitOutput() {
    uint8_t data[8] = {0};
    int16_t kp_tmp, kd_tmp, pos_tmp, vel_tmp, torque_tmp;

    if (state_.mode == MIT) {
        kp_tmp = DmMotorBase::float_to_uint(state_.kp_setpoint, KP_MIN, KP_MAX, 12);
        kd_tmp = DmMotorBase::float_to_uint(state_.kd_setpoint, KD_MIN, KD_MAX, 12);
        pos_tmp = DmMotorBase::float_to_uint(state_.position_setpoint, P_MIN, P_MAX, 16);
        vel_tmp = DmMotorBase::float_to_uint(state_.velocity_setpoint, V_MIN, V_MAX, 12);
        torque_tmp = DmMotorBase::float_to_uint(state_.torque_feedforward, T_MIN, T_MAX, 12);
        data[0] = pos_tmp >> 8;
        data[1] = pos_tmp & 0x00ff;
        data[2] = (vel_tmp >> 4) & 0x00ff;
        data[3] = ((vel_tmp & 0x000f) << 4) | ((kp_tmp >> 8) & 0x000f);
        data[4] = kp_tmp & 0x00ff;
        data[5] = (kd_tmp >> 4) & 0x00ff;
        data[6] = ((kd_tmp & 0x000f) << 4) | ((torque_tmp >> 8) & 0x000f);
        data[7] = torque_tmp & 0x00ff;
    } else if (state_.mode == POS_VEL) {
        uint8_t *pbuf, *vbuf;
        pbuf = (uint8_t*)&state_.position_setpoint;
        vbuf = (uint8_t*)&state_.velocity_setpoint;
        data[0] = *pbuf;
        data[1] = *(pbuf + 1);
        data[2] = *(pbuf + 2);
        data[3] = *(pbuf + 3);
        data[4] = *vbuf;
        data[5] = *(vbuf + 1);
        data[6] = *(vbuf + 2);
        data[7] = *(vbuf + 3);
    } else if (state_.mode == VEL) {
        uint8_t* vbuf;
        vbuf = (uint8_t*)&state_.velocity_setpoint;
        data[0] = *vbuf;
        data[1] = *(vbuf + 1);
        data[2] = *(vbuf + 2);
        data[3] = *(vbuf + 3);
    } else {
        RM_EXPECT_TRUE(false, "Invalid mode number!");
    }
    state_.can->Transmit(state_.tx_id_actual, data, 8);
}

void DMMotor4310::PrintData() const {
    set_cursor(0, 0);
    clear_screen();
    print("Position: % .4f ", GetTheta());
    print("Velocity: % .4f ", GetOmega());
    print("Torque: % .4f ", GetTorque());
    print("Rotor temp: % .4f \r\n", state_.rotor_temperature);
}

}  // namespace driver
