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

#include <cstring>

#include "arm_math.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "utils.h"

namespace driver {

// ===== DmRxFrame =====

void DmRxFrame::Load(const uint8_t data[8]) {
    motor_id = data[0] & 0x0f;
    status = static_cast<DmControlStatus>(data[0] >> 4);
    raw_theta = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);
    raw_omega = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4));
    raw_torque = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
    raw_mos_temp = data[6];
    raw_rotor_temp = data[7];
}

// ===== DmTxFrame =====

uint8_t DmTxFrame::Pack(uint8_t data[8], DmControlMode mode, float angle_max, float omega_max, float torque_max,
                        float kp_max, float kd_max) const {
    switch (mode) {
    case DmControlMode::MIT: {
        const float pos = clip(p_des, -angle_max, angle_max);
        const float vel = clip(v_des, -omega_max, omega_max);
        const float torque = clip(t_ff, -torque_max, torque_max);
        const float kp_clamped = clip(kp, 0.0f, kp_max);
        const float kd_clamped = clip(kd, 0.0f, kd_max);

        // 
        const uint16_t target_pos = signed_linear_remap(pos, 0x7FFF, 0xFFFF, angle_max);
        const uint16_t target_vel = signed_linear_remap(vel, 0x7FF, 0xFFF, omega_max);
        const uint16_t target_torque = signed_linear_remap(torque, 0x7FF, 0xFFF, torque_max);
        const uint16_t target_kp = static_cast<uint16_t>(linear_remap(kp_clamped, 0.0f, kp_max, 0.0f, 4095.0f));
        const uint16_t target_kd = static_cast<uint16_t>(linear_remap(kd_clamped, 0.0f, kd_max, 0.0f, 4095.0f));

        data[0] = static_cast<uint8_t>((target_pos >> 8) & 0xff);
        data[1] = static_cast<uint8_t>(target_pos & 0xff);
        data[2] = static_cast<uint8_t>((target_vel >> 4) & 0xff);
        data[3] = static_cast<uint8_t>(((target_vel & 0x000f) << 4) | ((target_kp >> 8) & 0x000f));
        data[4] = static_cast<uint8_t>(target_kp & 0xff);
        data[5] = static_cast<uint8_t>((target_kd >> 4) & 0xff);
        data[6] = static_cast<uint8_t>(((target_kd & 0x000f) << 4) | ((target_torque >> 8) & 0x000f));
        data[7] = static_cast<uint8_t>(target_torque & 0xff);
        return 8;
    }
    case DmControlMode::POS_VEL: {
        const float control_angle = clip(p_des, -angle_max, angle_max);
        const float control_omega = clip(v_des, -omega_max, omega_max);
        memcpy(data, &control_angle, sizeof(control_angle));
        memcpy(data + 4, &control_omega, sizeof(control_omega));
        return 8;
    }
    case DmControlMode::VEL: {
        const float control_omega = clip(v_des, -omega_max, omega_max);
        memcpy(data, &control_omega, sizeof(control_omega));
        return 4;
    }
    case DmControlMode::EMIT:
    default:
        return 0;
    }
}

// ===== Static member definitions =====

DmMotorBase* DmMotorBase::instances_[16] = {};
uint8_t DmMotorBase::instance_count_ = 0;
bool DmMotorBase::dm_thread_started_ = false;
bsp::Thread* DmMotorBase::dm_thread_ = nullptr;
uint32_t DmMotorBase::dm_output_period_us_ = 1000;

// ===== DmMotorBase =====

DmMotorBase::DmMotorBase(bsp::CAN* can, uint16_t master_id, uint16_t motor_can_id, DmControlMode mode,
                         const DmMotorConfig& config)
    : CanMotorBase(30) {
    can_ = can;
    rx_id_ = master_id;
    tx_id_ = motor_can_id;
    state_.mode = mode;
    config_ = config;

    // DM 电机 16-bit 编码器映射为单圈 [0, 2π]，多圈由 CanMotorBase::ProcessAngleTracking 累计
    state_.power_on_angle = -1;

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

    RM_ASSERT_TRUE(bsp::GetHighresTickMicroSec() != 0, "Highres timer not initialized");

    CanMotorBase::RegisterCanCallback(can, master_id, &DmMotorBase::RxThunk, this);
}

void DmMotorBase::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<DmMotorBase*>(ctx)->UpdateData(data);
}

void DmMotorBase::UpdateData(const uint8_t data[]) {
    state_.rx.Load(data);
    if (state_.rx.motor_id != (tx_id_ & 0x0f)) {
        return;
    }
    //state_.theta = signed_linear_remap(state_.rx.raw_theta, 0x7FFF, 0xFFFF, config_.angle_max);
    state_.theta = linear_remap<uint16_t, float>(state_.rx.raw_theta, 0, 65535, 0.0f, 2 * PI);

    state_.omega = signed_linear_remap(state_.rx.raw_omega, 0x7FF, 0xFFF, config_.omega_max);
    state_.torque = signed_linear_remap(state_.rx.raw_torque, 0x7FF, 0xFFF, config_.torque_max);    state_.feedback_pending = true;
}

void DmMotorBase::FinishFeedbackUpdate() {
    AngleTrackingContext ctx{
        state_.theta,
        state_.omega,
        state_.output_shaft_theta,
        state_.output_shaft_omega,
        state_.power_on_angle,
        state_.encoder_relative_angle,
        state_.encoder_cumulated_turns,
        state_.encoder_cumulated_angle,
        state_.output_relative_angle,
        state_.output_cumulated_turns,
        state_.output_cumulated_angle,
        config_.transmission_ratio,
        state_.absolute_mode,
    };
    CanMotorBase::FinishFeedbackUpdate(ctx);
}

float DmMotorBase::GetTheta() const {
    return state_.theta;
}

float DmMotorBase::GetOmega() const {
    return state_.omega;
}

float DmMotorBase::GetOutputShaftTheta() const {
    return state_.output_shaft_theta;
}

float DmMotorBase::GetOutputShaftOmega() const {
    return state_.output_shaft_omega;
}

void DmMotorBase::Enable() {
    state_.enable = true;
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    CanMotorBase::TransmitFrame(can_, tx_id_, data);
}

void DmMotorBase::Disable() {
    state_.enable = false;
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    CanMotorBase::TransmitFrame(can_, tx_id_, data);
}

bool DmMotorBase::IsEnable() const {
    return state_.enable;
}

int16_t DmMotorBase::GetOutput() {
    return 0;
}

void DmMotorBase::SetOutput(int16_t val) {
    (void)val;
    RM_ASSERT_FALSE(true, "DM motor does not support SetOutput(int16_t); use SetTarget() or SetOutput(float)");
}

void DmMotorBase::SetZeroPos() {
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
    CanMotorBase::TransmitFrame(can_, tx_id_, data);
}

void DmMotorBase::ClearError() {
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
    CanMotorBase::TransmitFrame(can_, tx_id_, data);
}

void DmMotorBase::SetFrequency(uint32_t freq) {
    RM_ASSERT_GT(freq, 0, "Frequency must be positive");
    dm_output_period_us_ = 1000000 / freq;
}

void DmMotorBase::SetMode(DmControlMode mode) {
    state_.mode = mode;
    tx_id_ = tx_id_ + static_cast<uint16_t>(mode);
}

DmControlStatus DmMotorBase::GetControlStatus() const {
    return state_.rx.status;
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
    if (state_.feedback_pending) {
        FinishFeedbackUpdate();
        state_.feedback_pending = false;
    }

    if (!state_.enable) {
        return;
    }

    switch (state_.rx.status) {
    case DmControlStatus::ENABLE:
        TransmitOutput();
        break;
    case DmControlStatus::DISABLE:
        Enable();
        break;
    default:
        ClearError();
        Enable();
        break;
    }
}

void DmMotorBase::SetTarget(float target, bool override) {
    (void)override;
    switch (state_.mode) {
    case DmControlMode::VEL:
        state_.tx.SetVel(target);
        break;
    case DmControlMode::MIT:
    case DmControlMode::POS_VEL:
        state_.tx.p_des = target;
        break;
    default:
        RM_ASSERT_TRUE(false, "SetTarget not supported in current DM control mode");
        break;
    }
}

void DmMotorBase::SetPosParams(float velocity) {
    RM_ASSERT_TRUE(state_.mode == DmControlMode::POS_VEL, "SetSpeedOffset only valid in POS_VEL mode");
    state_.tx.v_des = velocity;
}

void DmMotorBase::SetMitParams(float velocity, float kp, float kd, float t_ff) {
    RM_ASSERT_TRUE(state_.mode == DmControlMode::MIT, "SetMitParams only valid in MIT mode");
    state_.tx.v_des = velocity;
    state_.tx.kp = kp;
    state_.tx.kd = kd;
    state_.tx.t_ff = t_ff;
}

float DmMotorBase::GetTorque() const {
    return state_.torque;
}

void DmMotorBase::TransmitOutput() {
    uint8_t data[8] = {0};
    const uint8_t dlc = state_.tx.Pack(data, state_.mode, config_.angle_max, config_.omega_max, config_.torque_max,
                                       config_.kp_max, config_.kd_max);
    if (dlc == 0) {
        RM_EXPECT_TRUE(false, "Unsupported DM control mode");
        return;
    }

    CanMotorBase::TransmitFrame(can_, tx_id_, data, dlc);
}

// ===== DMMotor4310 =====

DMMotor4310::DMMotor4310(bsp::CAN* can, uint16_t master_id, uint16_t motor_can_id, DmControlMode mode)
    : DmMotorBase(can, master_id, motor_can_id, mode, DmMotor4310Config::J4310_Config) {}

    
void DMMotor4310::PrintData() const {
    set_cursor(0, 0);
    clear_screen();

    print("=== DM4310 ===\r\n");
    print("online:%d enable:%d abs:%d pending:%d\r\n", IsOnline(), state_.enable, state_.absolute_mode,
          state_.feedback_pending);
    print("mode:%u status:%u rx_id:0x%03X tx_id:0x%03X last_us:%u\r\n", static_cast<uint16_t>(state_.mode),
          static_cast<uint8_t>(GetControlStatus()), rx_id_, tx_id_, last_uptime_microsec_);

    print("--- RX raw ---\r\n");
    print("motor_id:%u raw_theta:%u raw_omega:%u raw_torque:%u\r\n", state_.rx.motor_id, state_.rx.raw_theta,
          state_.rx.raw_omega, state_.rx.raw_torque);
    print("mos_temp:%u rotor_temp:%u\r\n", state_.rx.raw_mos_temp, state_.rx.raw_rotor_temp);

    print("--- Feedback ---\r\n");
    print("theta:% .4f omega:% .4f torque:% .4f\r\n", GetTheta(), GetOmega(), GetTorque());
    print("out_theta:% .4f out_omega:% .4f\r\n", GetOutputShaftTheta(), GetOutputShaftOmega());

    print("--- Angle track ---\r\n");
    print("pwr_on:% .4f enc_rel:% .4f enc_turns:% .2f enc_cum:% .4f\r\n", state_.power_on_angle,
          state_.encoder_relative_angle, state_.encoder_cumulated_turns, state_.encoder_cumulated_angle);
    print("out_rel:% .4f out_turns:% .2f out_cum:% .4f\r\n", state_.output_relative_angle,
          state_.output_cumulated_turns, state_.output_cumulated_angle);

    print("--- TX target ---\r\n");
    print("p_des:% .4f v_des:% .4f kp:% .2f kd:% .2f t_ff:% .4f\r\n", state_.tx.p_des, state_.tx.v_des,
          state_.tx.kp, state_.tx.kd, state_.tx.t_ff);
}

}  // namespace driver
