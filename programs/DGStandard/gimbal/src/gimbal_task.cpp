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


osThreadId_t gimbalTaskHandle;

driver::Motor6020* pitch_motor = nullptr;
driver::Motor6020* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
INS_Angle_t INS_Angle;

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
    for (int i = 0; i < 1500; ++i) {
        check_kill();
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
    }

    Buzzer_Sing(SingCaliStart);
    imu->Calibrate();
    while (!imu->CaliDone()) {
        check_kill();
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
    }
    Buzzer_Sing(SingCaliDone);
    osDelay(100);

    // 每轮最大增加 0.0002 rad；1 ms 周期下约为 11.5 度/秒。
    const float remote_ratio = 0.0002f;
    const int rocker_deadband = 10;
    while (true) {
        check_kill();
        INS_Angle.pitch = imu->INS_angle[1];
        INS_Angle.yaw = imu->INS_angle[0];
        INS_Angle.roll = imu->INS_angle[2];

        pitch_diff = 0;
        yaw_diff = 0;
        if (dbus->ch3 > rocker_deadband || dbus->ch3 < -rocker_deadband) {
            pitch_diff = clip<float>((float)dbus->ch3 / dbus->ROCKER_MAX, -1.0f, 1.0f) * remote_ratio;
        }
        if (dbus->ch2 > rocker_deadband || dbus->ch2 < -rocker_deadband) {
            yaw_diff = clip<float>((float)dbus->ch2 / dbus->ROCKER_MAX, -1.0f, 1.0f) * remote_ratio;
        }

        // 先验证编码器位置控制；IMU、自瞄和底盘前馈均不修改电机目标。
        gimbal->TargetRel(pitch_diff, yaw_diff);
        gimbal->Update();
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
        .max_out = 0.5f,  // 调试阶段：角度环要求的速度最多 0.5 rad/s
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
        .ki = 2,  // 每次速度环计算累加；低速时逐步补足重力负载所需的输出
        .kd = 0,
        // 原参数在静止时最多只有 500 * 0.5 + 2500 = 2750，实车无法回位。
        // 保持角度环 0.5 rad/s 的速度指令上限，增加可用输出；仍须实车验证。
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
        {.kp = 8,
         .ki = 0,
         .kd = 0,  // 再大会在前面顿一下
         .max_out = 0.5f,  // 与 Pitch 一样限制回位速度指令，增强输出不依赖提高速度
         .max_iout = PI / 8,
         .deadband = PI / 180,
         .A = 0,                                    // 变速积分所能达到的最大值为A+B
         .B = 0,                                    // 启动变速积分的死区
         .output_filtering_coefficient = 0.5,       // 输出滤波系数
         .derivative_filtering_coefficient = 0.05,  // 微分滤波系数
         .mode = control::ConstrainedPID::OutputFilter | control::ConstrainedPID::DerivativeFilter |
                 control::ConstrainedPID::Integral_Limit};
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::DjiMotorBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 1500,  // 恢复上一轮可运动版本的速度环比例
        .ki = 1,  // 持续存在误差时逐步克服静摩擦
        .kd = 0,
        .max_out = 4000,  // 本车低速负载测试限幅
        .max_iout = 3000,  // 为 0 会把积分项清零，不能补足静摩擦负载
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

inline bool gimbal_en() {
    return dbus->IsOnline() &&
           (dbus->swr == remote::UP || dbus->swr == remote::MID) &&
           remote_mode == REMOTE_MODE_FOLLOW &&
           pitch_motor->IsOnline() && yaw_motor->IsOnline();
}

void check_kill() {
    while (!gimbal_en()) {
        yaw_motor->Disable();
        pitch_motor->Disable();
        osDelay(1);
    }
    yaw_motor->Enable();
    pitch_motor->Enable();
}
