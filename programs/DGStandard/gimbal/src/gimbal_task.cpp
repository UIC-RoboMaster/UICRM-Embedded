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

#include "gimbal_task.h"


#include "dji_remote.h"
#include "remote_control.h"


osThreadId_t gimbalTaskHandle;

driver::Motor6020* pitch_motor = nullptr;
driver::Motor6020* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
INS_Angle_t INS_Angle;
const char* gimbal_phase = "STARTUP";
float yaw_peak_speed = 0;
int yaw_peak_output = 0;
uint32_t gimbal_inhibit_mask = 0;
uint32_t gimbal_last_inhibit_mask = 0;
uint32_t gimbal_disable_events = 0;

control::gimbal_t gimbal_data;

void check_kill();

float pitch_feedforward;

void gimbalTask(void* arg) {
    UNUSED(arg);
    pitch_motor->Disable();
    yaw_motor->Disable();
    pitch_motor->SetSpeedFeedforward(0);
    yaw_motor->SetSpeedFeedforward(0);
    osDelay(1500);

    // 保留现有启动回中；只有 DBUS 和两个电机反馈正常时才允许输出。
    gimbal_phase = "CENTER";
    for (int i = 0; i < 1500; ++i) {
        check_kill();
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
    }

    Buzzer_Sing(SingCaliStart);
    gimbal_phase = "IMU_CALI";
    imu->Calibrate();
    while (!imu->CaliDone()) {
        check_kill();
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
    }
    Buzzer_Sing(SingCaliDone);
    osDelay(100);

    // 单位为 rad/s，使用实际任务间隔，避免调度快慢改变遥控灵敏度。
    const float pitch_remote_speed = 1.5f;
    const float yaw_remote_speed = 2.0f;
    const float yaw_target_lead_limit = PI / 12;  // 移动时目标最多领先实际位置 15 度。
    const int rocker_deadband = 10;
    // 左手同时操作 yaw 时 ch3 会带入 60~80 的串动，用大死区滤掉（实测数据见日志）
    const int pitch_rocker_deadband = 100;
    const float tick_seconds = 1.0f / osKernelGetTickFreq();
    uint32_t previous_tick = osKernelGetTickCount();
    int yaw_last_direction = 0;
    gimbal_phase = "REMOTE";
    while (true) {
        check_kill();
        const uint32_t now = osKernelGetTickCount();
        // 掉线/关闭后重新使能，不补算等待期间的摇杆输入。
        const float dt = clip<float>((now - previous_tick) * tick_seconds, 0, 0.005f);
        previous_tick = now;
        INS_Angle.pitch = imu->INS_angle[1];
        INS_Angle.yaw = imu->INS_angle[0];
        INS_Angle.roll = imu->INS_angle[2];

        pitch_diff = dgstandard::NormalizeRocker(dbus->ch3, dbus->ROCKER_MAX, pitch_rocker_deadband) *
                     pitch_remote_speed * dt;
        float yaw_input = dgstandard::NormalizeRocker(dbus->ch2, dbus->ROCKER_MAX, rocker_deadband);
        if (gimbal_param->yaw_inverted)
            yaw_input = -yaw_input;
        const float yaw_error = wrapc<float>(yaw_motor->GetTarget() - yaw_motor->GetTheta(), -PI, PI);
        const auto yaw_step = dgstandard::AdvanceYawTarget(
            yaw_error, yaw_input, dt, yaw_remote_speed, yaw_target_lead_limit, yaw_last_direction);
        yaw_last_direction = yaw_step.direction;
        yaw_diff = gimbal_param->yaw_inverted ? -yaw_step.delta : yaw_step.delta;
        if (yaw_step.reset_integral)
            yaw_motor->RequestPIDIntegralReset(driver::DjiMotorBase::OMEGA);

        // 先验证编码器位置控制；IMU、自瞄和底盘前馈均不修改电机目标。
        gimbal->TargetRel(pitch_diff, yaw_diff);
        gimbal->Update();
        // 比 USB 打印更频繁地保留峰值，避免 200ms 打印漏掉短时突变。
        const float yaw_speed_abs = fabsf(yaw_motor->GetOmega());
        const int yaw_output = yaw_motor->GetOutput();
        const int yaw_output_abs = yaw_output < 0 ? -yaw_output : yaw_output;
        if (yaw_speed_abs > yaw_peak_speed)
            yaw_peak_speed = yaw_speed_abs;
        if (yaw_output_abs > yaw_peak_output)
            yaw_peak_output = yaw_output_abs;
        osDelay(GIMBAL_OS_DELAY);
    }
}
void init_gimbal() {
    // 云台需要使用两个6020电机，并且进行角度环和速度环双环控制。此时我们需要初始化两个电机对象和四个PID对象，并且将四个PID对象分别绑定到两个电机对象上。

    /**
     * pitch motor
     */
    pitch_motor = new driver::Motor6020(can2, 0x209);
    pitch_motor->Disable();
    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 4,
        .ki = 0,
        .kd = 0,
        .max_out = 1.5f,  // Pitch 速度指令上限约为 85.9 度/秒
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 1,             // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::DjiMotorBase::THETA);
    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 3000,  // 实车已可回位；先降低 25% 试调，观察超调是否减轻
        .ki = 20,  // 负载变化（yaw 转动引起的重力分量变化）时快速补足输出，减轻 pitch 下沉
        .kd = 0,
        // 原参数在静止时最多只有 500 * 0.5 + 2500 = 2750，实车无法回位。
        // 速度指令上限见角度环；这里限制负载所需的输出。
        .max_out = 8000,
        .max_iout = 6000,
        .deadband = 0,                                                // 死区
        .A = 1.5 * PI,                                                // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                                  // 启动变速积分的死区
        .output_filtering_coefficient = 0.5,                          // 输出滤波系数
        .derivative_filtering_coefficient = 0.5,                      // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::DjiMotorBase::OMEGA);
    // 给电机启动角度环和速度环，并且这是一个绝对角度电机，需要启动绝对角度模式
    pitch_motor->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE);
    pitch_motor->SetSpeedFilter(0.1);

    /**
     * yaw motor
     */
    yaw_motor = new driver::Motor6020(can1, 0x209);
    yaw_motor->Disable();
    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_motor_theta_pid_init =
        {.kp = 12,  // 提高追赶强度；换向纠偏由 lead limit 与积分复位机制控制
         .ki = 0,
         .kd = 0,  // 再大会在前面顿一下
         .max_out = 2.0f,  // Yaw 速度指令上限约为 114.6 度/秒
         .max_iout = PI / 8,
         .deadband = PI / 1800,  // 0.1 度，减轻轻推时等待目标跨过死区的问题。
         .A = 0,                                    // 变速积分所能达到的最大值为A+B
         .B = 0,                                    // 启动变速积分的死区
         .output_filtering_coefficient = 0.5,       // 输出滤波系数
         .derivative_filtering_coefficient = 0.05,  // 微分滤波系数
         .mode = control::ConstrainedPID::OutputFilter | control::ConstrainedPID::DerivativeFilter |
                 control::ConstrainedPID::Integral_Limit};
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::DjiMotorBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 1000,  // 恢复上一轮可运动版本的速度环比例
        .ki = 80,  // 快速建立克服静摩擦的电流（原 1 需数秒才攒够 2000，导致起步延迟）
        .kd = 0,
        .max_out = 4000,  // 本车低速负载测试限幅
        .max_iout = 2000,  // 为 0 会把积分项清零，不能补足静摩擦负载
        .deadband = 0,    // 低速位置测试：避免小速度指令被死区清零
        .A = 0.5 * PI,    // 变速积分所能达到的最大值为A+B
        .B = 0.5 * PI,    // 启动变速积分的死区
        .output_filtering_coefficient = 0.5,
        .derivative_filtering_coefficient = 0.0003,                   // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    yaw_motor->ReInitPID(yaw_motor_omega_pid_init, driver::DjiMotorBase::OMEGA);
    // 给电机启动角度环和速度环，并且这是一个绝对角度电机，需要启动绝对角度模式
    yaw_motor->SetSpeedFilter(0.1);
    yaw_motor->SetSpeedCommandAcceleration(10.0f);  // 速度指令约 0.2s 内到位，减少起步附加延迟
    yaw_motor->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE);


    // 初始化云台对象
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
    // 在使能前先写入回中目标，避免短暂追踪默认的编码器 0。
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
}

inline uint32_t gimbal_inhibit_reason() {
    uint32_t reason = 0;
    if (!dbus->IsOnline())
        reason |= 1;
    if (dbus->swr != remote::UP && dbus->swr != remote::MID)
        reason |= 2;
    if (remote_mode != REMOTE_MODE_FOLLOW)
        reason |= 4;
    if (!pitch_motor->IsOnline())
        reason |= 8;
    if (!yaw_motor->IsOnline())
        reason |= 16;
    return reason;
}

void check_kill() {
    uint32_t reason = gimbal_inhibit_reason();
    bool counted = false;
    while (reason != 0) {
        if (!counted) {
            ++gimbal_disable_events;
            gimbal_last_inhibit_mask = 0;
            counted = true;
        }
        gimbal_inhibit_mask = reason;
        gimbal_last_inhibit_mask |= reason;
        yaw_motor->Disable();
        pitch_motor->Disable();
        osDelay(1);
        reason = gimbal_inhibit_reason();
    }
    gimbal_inhibit_mask = 0;
    yaw_motor->Enable();
    pitch_motor->Enable();
}
