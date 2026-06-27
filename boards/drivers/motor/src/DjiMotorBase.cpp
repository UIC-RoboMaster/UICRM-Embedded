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

#include "DjiMotorBase.h"

#include "arm_math.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "utils.h"

using namespace bsp;

namespace driver {

bool DjiMotorBase::is_init_ = false;
// 一个 group = (TX ID, CAN) 二元组。DJI 仅 3 个 TX ID，[10] 为预留值。
// 同组电机共享一帧 CAN 报文（最多 4 个），不同 (TX ID, CAN) 即新建 group。
DjiMotorBase::MotorGroup DjiMotorBase::groups_[10] = {};
uint8_t DjiMotorBase::group_count_ = 0;

bsp::Thread* DjiMotorBase::can_motor_thread_ = nullptr;
uint32_t DjiMotorBase::delay_time = 1;

DjiMotorBase::callback_t DjiMotorBase::pre_output_callback_ = [](void* args) { UNUSED(args); };
DjiMotorBase::callback_t DjiMotorBase::post_output_callback_ = [](void* args) { UNUSED(args); };
void* DjiMotorBase::pre_output_callback_instance_ = nullptr;
void* DjiMotorBase::post_output_callback_instance_ = nullptr;

DjiMotorBase::DjiMotorBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
    : MotorCANBase<DjiMotorBase>(30) {
    state_.can = can;
    state_.rx_id = rx_id;
    // 大疆的电机，自动识别 TX_ID 或使用显式指定的值
    if (tx_id == 0x00) {
        constexpr uint16_t GROUP_SIZE = 4;
        constexpr uint16_t RX1_ID_START = 0x201;
        constexpr uint16_t RX2_ID_START = 0x205;
        constexpr uint16_t RX3_ID_START = 0x209;
        constexpr uint16_t TX1_ID = 0x200;
        constexpr uint16_t TX2_ID = 0x1ff;
        constexpr uint16_t TX3_ID = 0x2ff;

        RM_ASSERT_GE(rx_id, RX1_ID_START, "Invalid rx id");
        RM_ASSERT_LT(rx_id, RX3_ID_START + GROUP_SIZE, "Invalid rx id");
        if (rx_id >= RX3_ID_START)
            state_.tx_id = TX3_ID;
        else if (rx_id >= RX2_ID_START)
            state_.tx_id = TX2_ID;
        else
            state_.tx_id = TX1_ID;
    } else {
        state_.tx_id = tx_id;
    }

    // 如果是第一次初始化，需要创建一个后台线程以固定频率输出电机指令
    if (!is_init_) {
        is_init_ = true;
        bsp::thread_init_t thread_init = {
            .func = CanMotorThread, .args = nullptr, .attr = can_motor_thread_attr_};
        can_motor_thread_ = new bsp::Thread(thread_init);
        can_motor_thread_->Start();
        memset(&groups_, 0, sizeof(groups_));
        for (uint8_t k = 0; k < 10; k++) groups_[k].tx_id = 0xFFFF;
        group_count_ = 0;
    }
    // 如果已经初始化，需要检查是否有重复的 ID，如果没有则加入到数组以使后台线程能够持续给电机输出数据
    for (uint8_t i = 0; i < 10; i++) {
        if (state_.tx_id == groups_[i].tx_id && groups_[i].can == state_.can) {
            if (groups_[i].count < 4) {
                groups_[i].motors[groups_[i].count] = this;
                groups_[i].count++;
                break;
            } else {
                RM_ASSERT_TRUE(false, "Exceeding maximum of 4 motor commands per CAN message");
            }
        } else if (groups_[i].tx_id == 0xFFFF) {
            groups_[i].tx_id = state_.tx_id;
            groups_[i].can = state_.can;
            groups_[i].motors[0] = this;
            group_count_++;
            groups_[i].count++;
            break;
        }
    }

    // 默认 PID 参数
    omega_pid_ = control::ConstrainedPID();
    theta_pid_ = control::ConstrainedPID();

    // Check if the high resolution timer is initialized
    RM_ASSERT_TRUE(bsp::GetHighresTickMicroSec() != 0, "Highres timer not initialized");
}

void DjiMotorBase::SetFrequency(uint32_t freq) {
    // 频率设置必须在电机初始化之前
    RM_ASSERT_FALSE(is_init_, "Frequency should be set before motor initialization");
    // 通过频率设置每秒输出时的延迟时间
    delay_time = 1000 / freq;
}

void DjiMotorBase::TransmitOutput(DjiMotorBase* motors[], uint8_t num_motors) {
    uint8_t data[8] = {0};

    RM_ASSERT_GT(num_motors, 0, "Meaningless empty can motor transmission");
    RM_ASSERT_LE(num_motors, 4, "Exceeding maximum of 4 motor commands per CAN message");
    // 获取输出的数据到缓冲区
    for (uint8_t i = 0; i < num_motors; ++i) {
        RM_ASSERT_EQ(motors[i]->state_.tx_id, motors[0]->state_.tx_id, "tx id mismatch");
        RM_ASSERT_EQ(motors[i]->state_.can, motors[0]->state_.can, "can line mismatch");
        const uint8_t motor_idx = (motors[i]->state_.rx_id - 1) % 4;
        const int16_t output = motors[i]->output_;
        data[2 * motor_idx] = output >> 8;
        data[2 * motor_idx + 1] = output & 0xff;
    }
    // 发送数据
    motors[0]->state_.can->Transmit(motors[0]->state_.tx_id, data, 8);
}

void DjiMotorBase::CanMotorThread(void* args) {
    UNUSED(args);
    // 后台线程，用于持续输出电机指令
    while (1) {
        // 遍历所有的电机组，对每个组的电机进行输出
        for (uint8_t i = 0; i < group_count_; i++) {
            // 计算每个组的电机的 PID 输出
            for (uint8_t j = 0; j < groups_[i].count; j++) {
                groups_[i].motors[j]->CalcOutput();
            }
        }
        pre_output_callback_(pre_output_callback_instance_);
        for (uint8_t i = 0; i < group_count_; i++) {
            // 输出电机指令
            TransmitOutput(groups_[i].motors, groups_[i].count);
        }
        post_output_callback_(post_output_callback_instance_);
        osDelay(delay_time);
    }
}

void DjiMotorBase::UpdateData(const uint8_t data[]) {
    UNUSED(data);
    ProcessAngleTracking();
}

void DjiMotorBase::UpdateHoldingState() {
    if (state_.mode & THETA) {
        float diff = state_.target - GetOutputShaftTheta();
        if (state_.mode & ABSOLUTE)
            diff = wrap<float>(diff, -PI, PI);
        state_.holding = abs(diff) < state_.proximity_in;
    }
}

void DjiMotorBase::SetTarget(float target, bool override) {
    // 设置目标值
    // 目标值的单位取决于电机的模式
    // 如果电机启动了角度环PID，则目标值为角度，单位为Rad
    // 如果电机没启动角度环PID的情况下启动了速度环PID，则目标值为角速度，单位为Rad/s
    if (state_.mode & INVERTED) {
        target = -target;
    }

    // CURRENT 模式下不检查 holding（holding 是角度控制概念）
    if (!(state_.mode & CURRENT) && override == false && !state_.holding) {
        // 如果电机没有在 hold 状态，则不修改目标值
        return;
    }
    state_.target = target;

    // ABSOLUTE 模式下，认为输出轴只有一圈。
    if ((state_.mode & THETA) && (state_.mode & ABSOLUTE)) {
        state_.target = wrap<float>(state_.target, -PI, PI);
    }

    // 重新计算是否 Holding
    UpdateHoldingState();
}

void DjiMotorBase::CalcOutput() {
    if (!state_.enable) {
        // 如果电机被禁用，则清空 PID 积分项并输出 0
        SetOutput(0);
        // CURRENT 模式下不使用角度/速度 PID，无需清积分
        if (!(state_.mode & CURRENT)) {
            theta_pid_.ResetIntegral();
            omega_pid_.ResetIntegral();
        }
        return;
    }

    // 力矩控制模式：旁路角度/速度 PID，直接输出目标电流值
    if (state_.mode & CURRENT) {
        SetOutput((int16_t)state_.target);
        return;
    }

    float target = state_.target;

    // 最新收到的 CAN 包的时间戳
    uint32_t update_time_us = GetLastUptimeMicrosec();
        // 当前最新的CAN数据包的时间戳，和上次运行这个函数时，最新的CAN数据包的时间戳的差值
        // diff == 0 说明自从上次运行这个函数后没有收到新的CAN数据包
        // diff > 1500 说明收到了新的CAN数据包，但是因为丢包，距离上次收到的CAN数据包已经超过1.5ms
        uint32_t update_time_diff = update_time_us - state_.last_update_time_us;
        if (update_time_diff > 65535)
            update_time_diff += 65536;
        state_.last_update_time_us = update_time_diff;
    state_.motor_update_time_interval = 1000;
    uint32_t times = (update_time_diff + state_.motor_update_time_interval / 2) / state_.motor_update_time_interval;

    if (times == 0) {
            // print("Motor %x packet missing at %d\n", state_.rx_id, bsp::GetHighresTickMilliSec());
        return;
    }

    // 处理角度环 PID，输入角度差，输出速度值
    if (state_.mode & THETA) {
        if (state_.mode & ABSOLUTE) {
            // 在 ABSOLUTE 模式下，如果输出轴到目标要转动大于半圈，则从另一侧转过去
            if (target - state_.output_shaft_theta > PI)
                target = target - 2 * PI;
            if (target - state_.output_shaft_theta < -PI)
                target = target + 2 * PI;
        }
        target = theta_pid_.ComputeOutput(target, state_.output_shaft_theta);
    }

    // 对速度加上偏移量，前馈时使用
    target += state_.speed_offset;

    // 处理速度环 PID，输入速度差，输出电流值
    if (state_.mode & OMEGA) {
        target = omega_pid_.ComputeOutput(target, GetOutputShaftOmega());
    }

    // 输出
    if (state_.mode != NONE) {
        SetOutput((int16_t)target);
    }
}

void DjiMotorBase::ReInitPID(control::ConstrainedPID::PID_Init_t pid_init, uint8_t mode) {
    if (mode & OMEGA) {
        omega_pid_.Reinit(pid_init);
    } else if (mode & THETA) {
        theta_pid_.Reinit(pid_init);
    }
}

control::ConstrainedPID::PID_State_t DjiMotorBase::GetPIDState(uint8_t mode) const {
    if (mode & OMEGA && state_.mode & OMEGA) {
        return omega_pid_.State();
    } else if (mode & THETA && state_.mode & THETA) {
        return theta_pid_.State();
    }
    return control::ConstrainedPID::PID_State_t();
}

void DjiMotorBase::SetMode(uint8_t mode) {
    state_.mode = mode;
    // Sync absolute mode to base class
    SetAbsoluteMode(mode & ABSOLUTE);
}

float DjiMotorBase::GetTarget() const {
    return state_.target;
}

bool DjiMotorBase::IsHolding() const {
    return state_.holding;
}

void DjiMotorBase::Hold(bool override) {
    if (!IsHolding() && state_.mode & THETA) {
        SetTarget(GetOutputShaftTheta(), override);
    }
}

void DjiMotorBase::SetSpeedOffset(float offset) {
    state_.speed_offset = offset;
}

void DjiMotorBase::SetTorque(float torque_nm, bool override) {
    RM_ASSERT_TRUE(torque_constant_ > 0, "Torque constant not set for this motor");
    // torque_nm [N·m] → Amp → raw_current
    float raw_current = torque_nm * 1000.0f / (torque_constant_ * RAW_CURRENT_TO_AMP);
    SetTarget(raw_current, override);
}

float DjiMotorBase::GetTorque() const {
    // raw_current → Amp → N·m
    return state_.raw_current * RAW_CURRENT_TO_AMP * torque_constant_ / 1000.0f;
}

void DjiMotorBase::RegisterErrorCallback(DjiMotorBase::callback_t callback, void* instance) {
    error_callback_ = callback;
    error_callback_instance_ = instance;
    if (!(state_.mode & OMEGA)) {
            // 角度环才需要注册错误回调
        RM_ASSERT_TRUE(false, "Only theta mode motor can register error callback");
    } else {
        omega_pid_.RegisterErrorCallcack(ErrorCallbackWrapper, this);
    }
}

void DjiMotorBase::ErrorCallbackWrapper(void* instance,
                                        control::ConstrainedPID::PID_ErrorHandler_t type) {
    UNUSED(type);
    DjiMotorBase* motor = reinterpret_cast<DjiMotorBase*>(instance);
    if (motor->error_callback_ != nullptr)
        motor->error_callback_(motor);
}

void DjiMotorBase::RegisterPreOutputCallback(DjiMotorBase::callback_t callback,
                                             void* instance) {
    pre_output_callback_ = callback;
    pre_output_callback_instance_ = instance;
}

void DjiMotorBase::RegisterPostOutputCallback(DjiMotorBase::callback_t callback,
                                              void* instance) {
    post_output_callback_ = callback;
    post_output_callback_instance_ = instance;
}

// ===== Motor3508 =====
Motor3508::Motor3508(CAN* can, uint16_t rx_id) : DjiMotorBase(can, rx_id) {
    RegisterCanCallback();
}

void Motor3508::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = data[4] << 8 | data[5];
    state_.raw_temperature = data[6];

    constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
    constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
    state_.theta = state_.raw_theta * THETA_SCALE;
    state_.omega = state_.raw_omega * OMEGA_SCALE;

    ProcessAngleTracking();
}

void Motor3508::PrintData() const {
    print("online: %s ", (IsOnline() ? "true" : "false"));
    print("theta: % .4f ", GetTheta());
    print("output shaft theta: % .4f ", GetOutputShaftTheta());
    print("omega: % .4f ", GetOmega());
    print("output shaft omega: % .4f ", GetOutputShaftOmega());
    print("raw temperature: %3d ", state_.raw_temperature);
    print("raw current get: % d \r\n", state_.raw_current);
}

void Motor3508::SetOutput(int16_t val) {
    constexpr int16_t MAX_ABS_CURRENT = 12288;  // ~20A
    output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}

// ===== Motor6020 =====
Motor6020::Motor6020(CAN* can, uint16_t rx_id, uint16_t tx_id)
    : DjiMotorBase(can, rx_id, tx_id) {
    // 绝对位置电机不需要初始化 align_angle_
    state_.power_on_angle = 0;
    RegisterCanCallback();
}

void Motor6020::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = data[4] << 8 | data[5];
    state_.raw_temperature = data[6];

    constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
    constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
    state_.theta = state_.raw_theta * THETA_SCALE;
    state_.omega =
        (state_.raw_omega * OMEGA_SCALE) * input_speed_filter_ + state_.omega * (1 - input_speed_filter_);

    ProcessAngleTracking();
}

void Motor6020::PrintData() const {
    print("online: %s ", (IsOnline() ? "true" : "false"));
    print("theta: % .4f ", GetTheta());
    print("output shaft theta: % .4f ", GetOutputShaftTheta());
    print("omega: % .4f ", GetOmega());
    print("output shaft omega: % .4f ", GetOutputShaftOmega());
    print("raw temperature: %3d ", state_.raw_temperature);
    print("raw current get: % d \r\n", state_.raw_current);
}

void Motor6020::SetOutput(int16_t val) {
    constexpr int16_t MAX_ABS_CURRENT = 30000;
    output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}

void Motor6020::SetSpeedFilter(float ratio) {
    input_speed_filter_ = ratio;
}

// ===== Motor2006 =====
Motor2006::Motor2006(CAN* can, uint16_t rx_id) : DjiMotorBase(can, rx_id) {
    RegisterCanCallback();
}

void Motor2006::UpdateData(const uint8_t data[]) {
    state_.raw_theta = data[0] << 8 | data[1];
    state_.raw_omega = data[2] << 8 | data[3];
    state_.raw_current = data[4] << 8 | data[5];

    constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
    constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
    state_.theta = state_.raw_theta * THETA_SCALE;
    state_.omega = state_.raw_omega * OMEGA_SCALE;

    ProcessAngleTracking();
}

void Motor2006::PrintData() const {
    print("online: %s ", (IsOnline() ? "true" : "false"));
    print("theta: % .4f ", GetTheta());
    print("output shaft theta: % .4f ", GetOutputShaftTheta());
    print("omega: % .4f ", GetOmega());
    print("output shaft omega: % .4f ", GetOutputShaftOmega());
    print("raw current get: % d \r\n", state_.raw_current);
}

void Motor2006::SetOutput(int16_t val) {
    constexpr int16_t MAX_ABS_CURRENT = 10000;  // ~10A
    output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}


// ===== ServoMotor =====

static void servomotor_callback(const uint8_t data[], void* args) {
    ServoMotor* servo = reinterpret_cast<ServoMotor*>(args);
    servo->UpdateData(data);
}

ServoMotor::ServoMotor(servo_t data, float align_angle, float proximity_in,
                       float proximity_out) {
    motor_ = data.motor;
    max_speed_ = data.transmission_ratio * data.max_speed;
    max_acceleration_ = data.transmission_ratio * data.max_acceleration;
    transmission_ratio_ = data.transmission_ratio;
    proximity_in_ = proximity_in;
    proximity_out_ = proximity_out;

    hold_ = true;
    target_angle_ = 0;
    align_angle_ = align_angle;  // Wait for Update to initialize
    motor_angle_ = 0;
    offset_angle_ = 0;
    servo_angle_ = 0;
    cumulated_angle_ = 0;
    inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
    outer_wrap_detector_ = new FloatEdgeDetector(0, PI);
    hold_detector_ = new BoolEdgeDetector(false);

    omega_pid_.Reinit(data.omega_pid_param, data.max_iout, data.max_out);
    hold_pid_.Reinit(data.hold_pid_param, data.hold_max_iout, data.hold_max_out);

    // override original motor rx callback with servomotor callback
    data.motor->state_.can->RegisterRxCallback(data.motor->state_.rx_id, servomotor_callback, this);

    // Initially jam detection is not enabled
    jam_callback_ = nullptr;
    detect_head_ = -1;
    detect_period_ = -1;
    detect_total_ = 0;
    detect_buf_ = nullptr;
}

servo_status_t ServoMotor::SetTarget(const float target, bool override) {
    if (!hold_ && !override)
        return INPUT_REJECT;
    servo_status_t dir = target < target_angle_ ? TURNING_ANTICLOCKWISE : TURNING_CLOCKWISE;
    target_angle_ = target;
    return dir;
}

void ServoMotor::SetMaxSpeed(const float max_speed) {
    if (max_speed > 0)
        max_speed_ = transmission_ratio_ * max_speed;
    else
        RM_EXPECT_TRUE(false, "Max speed should be positive");
}

void ServoMotor::SetMaxAcceleration(const float max_acceleration) {
    if (max_acceleration > 0)
        max_acceleration_ = transmission_ratio_ * max_acceleration;
    else
        RM_EXPECT_TRUE(false, "Max acceleration should be positive");
}

void ServoMotor::CalcOutput() {
        // if holding status toggle, reseting corresponding pid to avoid error
        // building up
    hold_detector_->input(hold_);
    if (hold_detector_->edge()) {
        omega_pid_.Reset();
        hold_pid_.Reset();
    }

    if (hold_detector_->negEdge())
        start_time_ = GetHighresTickMicroSec();

        // calculate desired output with pid
    int16_t command;
    float target_diff = (target_angle_ - servo_angle_ - cumulated_angle_) * transmission_ratio_;
        // v = sqrt(2 * a * d)
    uint64_t current_time = GetHighresTickMicroSec();
    if (!hold_) {
        float speed_max_start =
            (current_time - start_time_) / 10e6 * max_acceleration_ * transmission_ratio_;
        float speed_max_target = sqrt(2 * max_acceleration_ * abs(target_diff));
        float current_speed =
            speed_max_start > speed_max_target ? speed_max_target : speed_max_start;
        current_speed = clip<float>(current_speed, 0, max_speed_);
        command = omega_pid_.ComputeConstrainedOutput(
            motor_->GetOmegaDelta(sign<float>(target_diff, 0) * current_speed));
    } else {
        command = hold_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_diff * 50));
    }
    motor_->SetOutput(command);

        // jam detection mechanism
    if (detect_buf_ != nullptr) {
            // update rolling sum and circular buffer
        detect_total_ += command - detect_buf_[detect_head_];
        detect_buf_[detect_head_] = command;
        detect_head_ = detect_head_ + 1 < detect_period_ ? detect_head_ + 1 : 0;

            // detect if motor is jammed
        jam_detector_->input(abs(detect_total_) >= jam_threshold_);
        if (jam_detector_->posEdge()) {
            servo_jam_t data;
            data.speed = max_speed_ / transmission_ratio_;
            jam_callback_(this, data);
        }
    }
}

void ServoMotor::Hold(bool override) {
    if (!Holding()) {
        SetTarget(GetTheta(), override);
    }
}

bool ServoMotor::Holding() const {
    return hold_;
}

float ServoMotor::GetTarget() const {
    return target_angle_;
}

void ServoMotor::RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                     uint8_t detect_period) {
    constexpr int maximum_command = 32768;  // maximum command that a CAN motor can accept
    RM_ASSERT_TRUE(effort_threshold > 0 && effort_threshold <= 1,
                   "Effort threshold should between 0 and 1");
        // storing function pointer for future invocation
    jam_callback_ = callback;

        // create and initialize circular buffer
    detect_head_ = 0;
    detect_period_ = detect_period;
    detect_total_ = 0;
    if (detect_buf_ != nullptr)
        delete detect_buf_;
    detect_buf_ = new int16_t[detect_period];
    memset(detect_buf_, 0, detect_period);

        // calculate callback trigger threshold and triggering facility
    jam_threshold_ = maximum_command * effort_threshold * detect_period;
    jam_detector_ = new BoolEdgeDetector(false);
}

void ServoMotor::PrintData() const {
    print("Svo-align: % 10.6f ", align_angle_);
    print("Svo-theta: % 10.6f ", GetTheta());
    print("Svo-omega: % 10.6f ", GetOmega());
    print("Svo-target: % 10.6f ", target_angle_);
    if (hold_)
        print("Svo-status: holding ");
    else
        print("Svo-status: moving  ");
    motor_->PrintData();
}

float ServoMotor::GetTheta() const {
    return servo_angle_ + cumulated_angle_;
}

float ServoMotor::GetThetaDelta(const float target) const {
    return target - GetTheta();
}

float ServoMotor::GetOmega() const {
    return motor_->GetOmega() / transmission_ratio_;
}

float ServoMotor::GetOmegaDelta(const float target) const {
    return target - motor_->GetOmega() / transmission_ratio_;
}

void ServoMotor::UpdateData(const uint8_t data[]) {
    motor_->UpdateData(data);

        // TODO: change the align angle calibration method
        // This is a dumb method to get the align angle
    if (align_angle_ < 0)
        align_angle_ = motor_->GetTheta();

        // If motor angle is jumped from near 2PI to near 0, then wrap detecter will
        // sense a negative edge, which means that the motor is turning in positive
        // direction when crossing encoder boarder. Vice versa for motor angle jumped
        // from near 0 to near 2PI
    motor_angle_ = motor_->GetTheta() - align_angle_;
    inner_wrap_detector_->input(motor_angle_);
    if (inner_wrap_detector_->negEdge())
        offset_angle_ = wrap<float>(offset_angle_ + 2 * PI / transmission_ratio_, 0, 2 * PI);
    else if (inner_wrap_detector_->posEdge())
        offset_angle_ = wrap<float>(offset_angle_ - 2 * PI / transmission_ratio_, 0, 2 * PI);

    servo_angle_ = wrap<float>(offset_angle_ + motor_angle_ / transmission_ratio_, 0, 2 * PI);
    outer_wrap_detector_->input(servo_angle_);
    if (outer_wrap_detector_->negEdge())
        cumulated_angle_ += 2 * PI;
    else if (outer_wrap_detector_->posEdge())
        cumulated_angle_ -= 2 * PI;

        // determine if the motor should be in hold state
    float diff = abs(GetThetaDelta(target_angle_));
    if (!hold_ && diff < proximity_in_)
        hold_ = true;
    if (hold_ && diff > proximity_out_)
        hold_ = false;
}

// ===== SteeringMotor =====
SteeringMotor::SteeringMotor(steering_t data) {
    servo_t servo_data;
    servo_data.motor = data.motor;
    servo_data.max_speed = data.max_speed;
    servo_data.max_acceleration = data.max_acceleration;
    servo_data.transmission_ratio = data.transmission_ratio;
    servo_data.omega_pid_param = data.omega_pid_param;
    servo_data.max_iout = data.max_iout;
    servo_data.max_out = data.max_out;
    servo_ = new ServoMotor(servo_data, data.offset_angle);

    test_speed_ = data.test_speed;
    align_detect_func = data.align_detect_func;
    calibrate_offset = data.calibrate_offset;
    align_angle_ = 0;
    align_detector = new BoolEdgeDetector(false);
    align_complete_ = false;
}

float SteeringMotor::GetRawTheta() const {
    return servo_->GetTheta();
}

void SteeringMotor::PrintData() const {
    print("Str-align: %10.5f ", align_angle_);
    servo_->PrintData();
}

void SteeringMotor::TurnRelative(float angle) {
    servo_->SetTarget(servo_->GetTarget() + angle, true);
}

void SteeringMotor::TurnAbsolute(float angle) {
    servo_->SetTarget(angle);
}

bool SteeringMotor::AlignUpdate() {
    if (align_complete_) {
        servo_->SetTarget(align_angle_, true);
        servo_->CalcOutput();
        return true;
    } else if (align_detect_func()) {
        float current_theta = servo_->motor_->GetTheta();
        float offset = wrap<float>(servo_->align_angle_ - current_theta, -PI, PI);
        float current =
            (current_theta + offset - servo_->align_angle_) / servo_->transmission_ratio_ +
            servo_->offset_angle_ + servo_->cumulated_angle_;
        align_angle_ = current + calibrate_offset;
        align_complete_ = true;
        servo_->SetTarget(align_angle_, true);
        servo_->CalcOutput();
        return true;
    } else {
        servo_->motor_->SetOutput(servo_->omega_pid_.ComputeConstrainedOutput(
            servo_->motor_->GetOmegaDelta(test_speed_ * servo_->transmission_ratio_)));
    }
    return false;
}

void SteeringMotor::Update() {
    servo_->CalcOutput();
}

// ===== FlyWheelMotor =====
FlyWheelMotor::FlyWheelMotor(flywheel_t data) {
    motor_ = data.motor;
    max_speed_ = data.max_speed;
    target_speed_ = 0;
    is_inverted_ = data.is_inverted;
    omega_pid_ = control::PIDController(data.omega_pid_param);
}

void FlyWheelMotor::SetSpeed(float speed) {
    if (is_inverted_) {
        speed = -speed;
    }
    speed = clip<float>(speed, -max_speed_, max_speed_);
    target_speed_ = speed;
}

void FlyWheelMotor::CalcOutput() {
    motor_->SetOutput(
        omega_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_speed_)));
}

float FlyWheelMotor::GetTarget() const {
    if (is_inverted_) {
        return -target_speed_;
    } else {
        return target_speed_;
    }
}

void FlyWheelMotor::PrintData() const {
    print("Fly-target: %2.5f ", target_speed_);
    motor_->PrintData();
}

void FlyWheelMotor::UpdateData(const uint8_t data[]) {
    motor_->UpdateData(data);
}

float FlyWheelMotor::GetTheta() const {
    return motor_->GetTheta();
}

float FlyWheelMotor::GetThetaDelta(const float target) const {
    return motor_->GetThetaDelta(target);
}

float FlyWheelMotor::GetOmega() const {
    return motor_->GetOmega();
}

float FlyWheelMotor::GetOmegaDelta(const float target) const {
    return motor_->GetOmegaDelta(target);
}

}  // namespace driver
