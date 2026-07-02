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

// 发送电机输出前的回调函数
DjiMotorBase::callback_t DjiMotorBase::pre_output_callback_ = [](void* args) { UNUSED(args); };
void* DjiMotorBase::pre_output_callback_instance_ = nullptr;

// 发送电机输出后的回调函数
DjiMotorBase::callback_t DjiMotorBase::post_output_callback_ = [](void* args) { UNUSED(args); };
void* DjiMotorBase::post_output_callback_instance_ = nullptr;

DjiMotorBase::DjiMotorBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
    : MotorCANBase<DjiMotorBase>(30) {
    state_.can = can;
    state_.rx_id = rx_id;
    state_.tx_id = tx_id;

    // 如果是第一次初始化，需要创建一个后台线程以固定频率输出电机指令
    if (!is_init_) {
        is_init_ = true;
        bsp::thread_init_t thread_init = {
            .func = CanMotorThread, .args = nullptr, .attr = can_motor_thread_attr_};
        can_motor_thread_ = new bsp::Thread(thread_init);
        can_motor_thread_->Start();
        // 初始化电机组，清空所有槽位
        memset(&groups_, 0, sizeof(groups_));
        group_count_ = 0;
    }
    // 在已有 group 中查找 (tx_id, can) 匹配的组
    for (uint8_t i = 0; i < 10; i++) {
        if (groups_[i].occupied && groups_[i].tx_id == state_.tx_id && groups_[i].can == state_.can) {
            RM_ASSERT_LT(groups_[i].count, 4, "Exceeding maximum of 4 motor commands per CAN message");
            groups_[i].motors[groups_[i].count++] = this;
            break;
        }
        // 如果没有找到匹配的组，则占用一个新的槽位
        if (!groups_[i].occupied) {
            groups_[i].occupied = true;
            groups_[i].tx_id = state_.tx_id;
            groups_[i].can = state_.can;
            groups_[i].motors[0] = this;
            groups_[i].count = 1;
            group_count_++;
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

void DjiMotorBase::TransmitOutput(const MotorGroup& group) {
    // 初始化数据帧
    uint8_t data[8] = {0};

    for (uint8_t i = 0; i < group.count; ++i) {
        // 计算电机在数据帧中的索引位置
        const uint8_t motor_idx = (group.motors[i]->state_.rx_id - 1) % 4;
        // 获取电机的电流输出值
        const int16_t output = group.motors[i]->output_;
        // 将电流输出值拆分为高字节和低字节，并放入数据帧中
        data[2 * motor_idx] = output >> 8;
        data[2 * motor_idx + 1] = output & 0xff;
    }
    group.can->Transmit(group.tx_id, data, 8);
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
            TransmitOutput(groups_[i]);
        }
        post_output_callback_(post_output_callback_instance_);
        osDelay(delay_time);
    }
}

void DjiMotorBase::UpdateData(const uint8_t data[]) {
    UNUSED(data);
    // TODO 基类模板是否可用
    // RM_ASSERT_TRUE(false, "DjiMotorBase::UpdateData should be implemented by derived motor");
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
    // 当前最新的 CAN 数据包时间戳与上次处理的 CAN 数据包时间戳的差值
    // diff == 0 说明自从上次运行这个函数后没有收到新的 CAN 数据包
    // diff > 1500 说明收到新 CAN 数据包，但距离上一帧超过 1.5ms，可能存在丢包
    uint32_t update_time_diff = update_time_us - state_.last_update_time_us;
    // stm32 f4 为 uint16_t，stm32 f7/f1 为 uint32_t
    if (update_time_us < state_.last_update_time_us && update_time_diff > 65535)
        update_time_diff += 65536;
    // 更新最后处理的 CAN 数据包时间戳
    state_.last_update_time_us = update_time_us;
    // 设置 can 通讯间隔
    state_.motor_update_time_interval = 1000;
    // 计算自上次处理后收到的 CAN 数据包数量，可能存在丢包
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


}  // namespace driver
