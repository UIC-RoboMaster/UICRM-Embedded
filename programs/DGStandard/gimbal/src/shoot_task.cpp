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

#include "shoot_task.h"

driver::Motor3508* flywheel_left = nullptr;
driver::Motor3508* flywheel_right = nullptr;

driver::DjiMotorBase* steering_motor = nullptr;

// bsp::GPIO* shoot_key = nullptr;

bool jam_notify_flags = false;

void jam_callback(void* args) {
    driver::Motor2006* motor = static_cast<driver::Motor2006*>(args);
    jam_notify_flags = true;
    float target = motor->GetTarget();
    if (target > motor->GetOutputShaftTheta()) {
        float prev_target = motor->GetTarget() - 2 * PI / 32;
        motor->SetTarget(prev_target);
    } else {
        float prev_target = motor->GetTarget() + 2 * PI / 32;
        motor->SetTarget(prev_target);
    }
}

osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    Ease flywheel_output(0, 1);
    bool was_enabled = false;

    while (true) {
        const bool shoot_en = dbus->IsOnline() && remote_mode == REMOTE_MODE_FOLLOW &&
                              (dbus->swr == remote::UP || dbus->swr == remote::MID) &&
                              imu->CaliDone() && steering_motor->IsOnline();
        if (!shoot_en) {
            kill_shoot();
            flywheel_output = Ease(0, 1);
            was_enabled = false;
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (!was_enabled) {
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());
            steering_motor->Enable();
            flywheel_left->Enable();
            flywheel_right->Enable();
            was_enabled = true;
        }

        const bool flywheel_requested = shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
                                        shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED;
        // 保留原程序 SetOutput(900) 的原始输出量级，不将 900 改解释为 rad/s。
        flywheel_output.SetTarget(flywheel_requested ? 900 : 0);
        const float output = flywheel_output.Calc();
        flywheel_left->SetTarget(output);
        flywheel_right->SetTarget(output);

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING && flywheel_output.IsAtTarget() &&
            flywheel_left->IsOnline() && flywheel_right->IsOnline()) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
        }
        if (shoot_flywheel_mode != SHOOT_FRIC_MODE_PREPARED ||
            !flywheel_left->IsOnline() || !flywheel_right->IsOnline()) {
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (shoot_load_mode == SHOOT_MODE_SINGLE) {
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta() + 2 * PI / 8);
            // 无弹丸传感器时每个 SINGLE 指令只设置一次目标。
            shoot_load_mode = SHOOT_MODE_IDLE;
        } else if (shoot_load_mode == SHOOT_MODE_BURST) {
            // 上一个供弹步进到位后才继续，避免每毫秒无限累加目标。
            steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, false);
        } else if (shoot_load_mode == SHOOT_MODE_STOP || shoot_load_mode == SHOOT_MODE_DISABLE) {
            steering_motor->Hold();
        }
        osDelay(SHOOT_OS_DELAY);
    }
}
void init_shoot() {
    flywheel_left = new driver::Motor3508(can2, 0x202);
    flywheel_right = new driver::Motor3508(can2, 0x203);
    flywheel_left->Disable();
    flywheel_right->Disable();
    flywheel_left->SetTransmissionRatio(1);
    flywheel_right->SetTransmissionRatio(1);

    // 使用原始电流目标，使后台电机线程负责发送，避免 SetOutput 被速度 PID 覆盖。
    flywheel_left->SetMode(driver::DjiMotorBase::CURRENT | driver::DjiMotorBase::INVERTED);
    flywheel_right->SetMode(driver::DjiMotorBase::CURRENT);
    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);
    steering_motor = new driver::Motor2006(can1, 0x207);
    steering_motor->Disable();

    steering_motor->SetTransmissionRatio(36);
    control::ConstrainedPID::PID_Init_t steering_motor_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 2 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.2,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    steering_motor->ReInitPID(steering_motor_theta_pid_init, driver::DjiMotorBase::THETA);
    control::ConstrainedPID::PID_Init_t steering_motor_omega_pid_init = {
        .kp = 5000,
        .ki = 2,
        .kd = 0,
        .max_out = 10000,
        .max_iout = 4000,
        .deadband = 0,                                           // 死区
        .A = 3 * PI,                                             // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                                             // 启动变速积分的死区
        .output_filtering_coefficient = 0.2,                     // 输出滤波系数
        .derivative_filtering_coefficient = 0,                   // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::ErrorHandle,            // 错误处理

    };
    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::DjiMotorBase::OMEGA);
    steering_motor->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA);

    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);
}
void kill_shoot() {
    steering_motor->Disable();
    flywheel_left->Disable();
    flywheel_right->Disable();
    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
}
