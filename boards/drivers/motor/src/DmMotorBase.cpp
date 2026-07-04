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
    : CanMotorBase(50) {
    state_.can = can;
    state_.rx_id = rx_id;
    state_.tx_id = tx_id;
    // DM 电机使用绝对值编码器，无需等待上电角度
    // TODO ？？
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

    RM_ASSERT_TRUE(bsp::GetHighresTickMicroSec() != 0, "Highres timer not initialized");
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
    CanMotorBase::TransmitFrame(state_.can, state_.tx_id, data);
}

void DmMotorBase::Disable() {
    state_.enable = false;
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    CanMotorBase::TransmitFrame(state_.can, state_.tx_id, data);
}

bool DmMotorBase::IsEnable() const {
    return state_.enable;
}

int16_t DmMotorBase::GetOutput() {
    return 0;
}

void DmMotorBase::SetOutput(int16_t val) {
    state_.velocity_setpoint = static_cast<float>(val);
}

void DmMotorBase::SetZeroPos() {
    const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
    CanMotorBase::TransmitFrame(state_.can, state_.tx_id, data);
}

void DmMotorBase::SetFrequency(uint32_t freq) {
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

void DmMotorBase::TransmitOutput() {
    uint8_t data[8] = {0};
    int16_t kp_tmp, kd_tmp, pos_tmp, vel_tmp, torque_tmp;

    if (state_.mode == MIT) {
        kp_tmp = (int16_t)linear_remap(state_.kp_setpoint, KP_MIN, KP_MAX, 0.0f, (float)MIT_PARAM_MAX_RAW);
        kd_tmp = (int16_t)linear_remap(state_.kd_setpoint, KD_MIN, KD_MAX, 0.0f, (float)MIT_PARAM_MAX_RAW);
        pos_tmp = (int16_t)linear_remap(state_.position_setpoint, P_MIN, P_MAX, 0.0f, (float)POS_MAX_RAW);
        vel_tmp = (int16_t)linear_remap(state_.velocity_setpoint, V_MIN, V_MAX, 0.0f, (float)MIT_PARAM_MAX_RAW);
        torque_tmp = (int16_t)linear_remap(state_.torque_feedforward, T_MIN, T_MAX, 0.0f, (float)MIT_PARAM_MAX_RAW);
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
    CanMotorBase::TransmitFrame(state_.can, state_.tx_id, data);
}

// ===== DMMotor4310 =====

DMMotor4310::DMMotor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_mode_t mode)
    : DmMotorBase(can, rx_id, tx_id) {
    state_.mode = mode;
    // DM 电机的控制帧 CAN ID = 软件配置的 can_id + 模式偏移
    state_.tx_id = tx_id + mode;
    CanMotorBase::RegisterCanCallback(can, rx_id, &DMMotor4310::RxThunk, this);
}

void DMMotor4310::RxThunk(void* ctx, const uint8_t data[]) {
    static_cast<DMMotor4310*>(ctx)->UpdateData(data);
}

void DMMotor4310::UpdateData(const uint8_t data[]) {
    state_.error = data[0] >> 4;
    state_.id = data[0] & 0x0f;
    state_.raw_position = data[1] << 8 | data[2];
    state_.raw_velocity = data[3] << 4 | (data[4] & 0xf0) >> 4;
    state_.raw_torque = data[5] | (data[4] & 0x0f) << 8;
    state_.mos_temperature = data[6];
    state_.rotor_temperature = data[7];

    state_.theta = linear_remap(state_.raw_position, 0u, POS_MAX_RAW, P_MIN, P_MAX);
    state_.omega = linear_remap(state_.raw_velocity, 0u, MIT_PARAM_MAX_RAW, V_MIN, V_MAX);
    state_.torque = linear_remap(state_.raw_torque, 0u, MIT_PARAM_MAX_RAW, T_MIN, T_MAX);

    FinishFeedbackUpdate();
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
