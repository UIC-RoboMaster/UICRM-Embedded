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

namespace {

/// 传统模式 16-bit MIT 位置命令零点（达妙协议 0x7fff）
constexpr uint16_t MIT_POS_ZERO_RAW = 0x7fff;

/// 传统模式 12-bit 速度/力矩/Kp/Kd 零点（反馈与 MIT 发送共用 0x7ff）
constexpr uint16_t PARAM_12BIT_ZERO_RAW = 0x7ff;

/**
 * @brief 浮点物理量映射为整型 raw（与 RoboWalker Math_Float_To_Int 一致）
 */
uint16_t DmFloatToRaw(float x, float float_min, float float_max, int32_t int_min, int32_t int_max) {
    float tmp = (x - float_min) / (float_max - float_min);
    auto out = static_cast<int32_t>(tmp * static_cast<float>(int_max - int_min) + static_cast<float>(int_min));
    return static_cast<uint16_t>(out);
}

}  // namespace

// ===== DmRxFeedback =====

void DmRxFeedback::Load(const uint8_t data[8]) {
    motor_id = data[0] & 0x0f;
    status = static_cast<DmControlStatus>(data[0] >> 4);
    encoder = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);
    omega = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4));
    torque = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
    mos_temp = data[6];
    rotor_temp = data[7];
}

// ===== DmTxFrameMit / PosVel / Vel =====

void DmTxFrameMit::Pack(uint8_t data[8], float pos, float vel, float kp, float kd, float torque, float angle_max,
                        float omega_max, float torque_max, float kp_max, float kd_max, uint16_t pos_max_raw,
                        uint16_t mit_param_max_raw) {
    pos = clip(pos, -angle_max, angle_max);
    vel = clip(vel, -omega_max, omega_max);
    torque = clip(torque, -torque_max, torque_max);
    kp = clip(kp, 0.0f, kp_max);
    kd = clip(kd, 0.0f, kd_max);

    const uint16_t pos_raw =
        DmFloatToRaw(pos, 0.0f, angle_max, MIT_POS_ZERO_RAW, static_cast<int32_t>(pos_max_raw));
    const uint16_t vel_raw =
        DmFloatToRaw(vel, 0.0f, omega_max, PARAM_12BIT_ZERO_RAW, static_cast<int32_t>(mit_param_max_raw));
    const uint16_t torque_raw =
        DmFloatToRaw(torque, 0.0f, torque_max, PARAM_12BIT_ZERO_RAW, static_cast<int32_t>(mit_param_max_raw));
    const uint16_t kp_raw = DmFloatToRaw(kp, 0.0f, kp_max, 0, static_cast<int32_t>(mit_param_max_raw));
    const uint16_t kd_raw = DmFloatToRaw(kd, 0.0f, kd_max, 0, static_cast<int32_t>(mit_param_max_raw));

    data[0] = static_cast<uint8_t>((pos_raw >> 8) & 0xff);
    data[1] = static_cast<uint8_t>(pos_raw & 0xff);
    data[2] = static_cast<uint8_t>((vel_raw >> 4) & 0xff);
    data[3] = static_cast<uint8_t>(((vel_raw & 0x000f) << 4) | ((kp_raw >> 8) & 0x000f));
    data[4] = static_cast<uint8_t>(kp_raw & 0xff);
    data[5] = static_cast<uint8_t>((kd_raw >> 4) & 0xff);
    data[6] = static_cast<uint8_t>(((kd_raw & 0x000f) << 4) | ((torque_raw >> 8) & 0x000f));
    data[7] = static_cast<uint8_t>(torque_raw & 0xff);
}

void DmTxFramePosVel::Pack(uint8_t data[8], float pos, float vel, float angle_max, float omega_max) {
    const float control_angle = clip(pos, -angle_max, angle_max);
    const float control_omega = clip(vel, -omega_max, omega_max);
    memcpy(data, &control_angle, sizeof(control_angle));
    memcpy(data + 4, &control_omega, sizeof(control_omega));
}

void DmTxFrameVel::Pack(uint8_t data[4], float vel, float omega_max) {
    const float control_omega = clip(vel, -omega_max, omega_max);
    memcpy(data, &control_omega, sizeof(control_omega));
}

// ===== Static member definitions =====

DmMotorBase* DmMotorBase::instances_[16] = {};
uint8_t DmMotorBase::instance_count_ = 0;
bool DmMotorBase::dm_thread_started_ = false;
bsp::Thread* DmMotorBase::dm_thread_ = nullptr;
uint32_t DmMotorBase::dm_output_period_us_ = 1000;

// ===== DmMotorBase =====

DmMotorBase::DmMotorBase(bsp::CAN* can, uint16_t master_id, uint16_t motor_can_id, DmControlMode mode,
                         float angle_max, float omega_max, float torque_max, float kp_max, float kd_max,
                         uint16_t pos_max_raw, uint16_t mit_param_max_raw)
    : CanMotorBase(50) {
    can_ = can;
    rx_id_ = master_id;
    motor_can_id_ = motor_can_id;
    state_.mode = mode;
    tx_id_ = motor_can_id + static_cast<uint16_t>(mode);

    angle_max_ = angle_max;
    omega_max_ = omega_max;
    torque_max_ = torque_max;
    kp_max_ = kp_max;
    kd_max_ = kd_max;
    pos_max_raw_ = pos_max_raw;
    mit_param_max_raw_ = mit_param_max_raw;

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
}

void DmMotorBase::ParseFeedbackNormal() {
    state_.theta = linear_remap<uint16_t, float>(state_.rx.encoder, static_cast<uint16_t>(0), pos_max_raw_, 0.0f,
                                                 2 * PI);
    state_.omega = (state_.rx.omega - FEEDBACK_PARAM_ZERO_RAW) / static_cast<float>(FEEDBACK_PARAM_ZERO_RAW) *
                   omega_max_;
    state_.torque = (static_cast<int16_t>(state_.rx.torque) - FEEDBACK_PARAM_ZERO_RAW) /
                    static_cast<float>(FEEDBACK_PARAM_ZERO_RAW) * torque_max_;
}

void DmMotorBase::FinishFeedbackUpdate() {
    AngleTrackingContext ctx{
        state_.theta,
        state_.omega,
        state_.output_shaft_theta,
        state_.output_shaft_omega,
        state_.power_on_angle,
        state_.relative_angle,
        state_.output_cumulated_angle,
        state_.output_relative_angle,
        state_.transmission_ratio,
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
    CanMotorBase::TransmitFrame(can_, motor_can_id_, data);
}

void DmMotorBase::Disable() {
    state_.enable = false;
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    CanMotorBase::TransmitFrame(can_, motor_can_id_, data);
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
    CanMotorBase::TransmitFrame(can_, motor_can_id_, data);
}

void DmMotorBase::ClearError() {
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
    CanMotorBase::TransmitFrame(can_, motor_can_id_, data);
}

void DmMotorBase::SetFrequency(uint32_t freq) {
    RM_ASSERT_GT(freq, 0, "Frequency must be positive");
    dm_output_period_us_ = 1000000 / freq;
}

void DmMotorBase::SetMode(DmControlMode mode) {
    state_.mode = mode;
    tx_id_ = motor_can_id_ + static_cast<uint16_t>(mode);
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
        ParseFeedbackNormal();
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

void DmMotorBase::SetOutput(float position, float velocity, float kp, float kd, float torque) {
    state_.kp_setpoint = kp;
    state_.kd_setpoint = kd;
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
    state_.torque_setpoint = torque;
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
    RM_ASSERT_TRUE(state_.mode == DmControlMode::VEL, "SetTarget(float) only valid in VEL mode");
    state_.velocity_setpoint = target;
}

void DmMotorBase::SetTarget(float position, float velocity, float kp, float kd, float t_ff) {
    RM_ASSERT_TRUE(state_.mode == DmControlMode::MIT, "SetTarget(5 args) only valid in MIT mode");
    state_.kp_setpoint = kp;
    state_.kd_setpoint = kd;
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
    state_.torque_setpoint = t_ff;
}

void DmMotorBase::SetTarget(float position, float velocity) {
    RM_ASSERT_TRUE(state_.mode == DmControlMode::POS_VEL, "SetTarget(2 args) only valid in POS_VEL mode");
    state_.position_setpoint = position;
    state_.velocity_setpoint = velocity;
}

float DmMotorBase::GetTorque() const {
    return state_.torque;
}

void DmMotorBase::TransmitOutput() {
    uint8_t data[8] = {0};
    uint8_t dlc = 8;

    switch (state_.mode) {
    case DmControlMode::MIT:
        DmTxFrameMit::Pack(data, state_.position_setpoint, state_.velocity_setpoint, state_.kp_setpoint,
                           state_.kd_setpoint, state_.torque_setpoint, angle_max_, omega_max_, torque_max_, kp_max_,
                           kd_max_, pos_max_raw_, mit_param_max_raw_);
        break;
    case DmControlMode::POS_VEL:
        DmTxFramePosVel::Pack(data, state_.position_setpoint, state_.velocity_setpoint, angle_max_, omega_max_);
        break;
    case DmControlMode::VEL:
        dlc = 4;
        DmTxFrameVel::Pack(data, state_.velocity_setpoint, omega_max_);
        break;
    case DmControlMode::EMIT:
    default:
        RM_EXPECT_TRUE(false, "Unsupported DM control mode");
        return;
    }

    CanMotorBase::TransmitFrame(can_, tx_id_, data, dlc);
}

// ===== DMMotor4310 =====

DMMotor4310::DMMotor4310(bsp::CAN* can, uint16_t master_id, uint16_t motor_can_id, DmControlMode mode)
    : DmMotorBase(can, master_id, motor_can_id, mode, DmMotor4310Config::ANGLE_MAX, DmMotor4310Config::OMEGA_MAX,
                  DmMotor4310Config::TORQUE_MAX, DmMotor4310Config::KP_MAX, DmMotor4310Config::KD_MAX,
                  DmMotor4310Config::POS_MAX_RAW, DmMotor4310Config::MIT_PARAM_MAX_RAW) {
    state_.transmission_ratio = DmMotor4310Config::TRANSMISSION_RATIO;
    CanMotorBase::RegisterCanCallback(can, master_id, &DMMotor4310::RxThunk, this);
}

void DMMotor4310::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<DMMotor4310*>(ctx)->UpdateData(data);
}

void DMMotor4310::UpdateData(const uint8_t data[]) {
    state_.rx.Load(data);
    if (state_.rx.motor_id != (motor_can_id_ & 0x0f)) {
        return;
    }
    state_.feedback_pending = true;
}

void DMMotor4310::PrintData() const {
    set_cursor(0, 0);
    clear_screen();
    print("Position: % .4f ", GetTheta());
    print("Velocity: % .4f ", GetOmega());
    print("Torque: % .4f ", GetTorque());
    print("Rotor temp: % .4f \r\n", state_.rx.rotor_temp);
}

}  // namespace driver
