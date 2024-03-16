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

osThreadId_t gimbalTaskHandle;

driver::MotorCANBase* pitch_motor = nullptr;
driver::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
void gimbalTask(void* arg) {
    UNUSED(arg);
    pitch_motor->Disable();
    yaw_motor->Disable();
    osDelay(1500);
    while (remote_mode == REMOTE_MODE_KILL) {
        kill_gimbal();
        osDelay(GIMBAL_OS_DELAY);
    }
    int i = 0;
    while (i < 2000) {
        while (remote_mode == REMOTE_MODE_KILL) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
        }
        if (!pitch_motor->IsEnable())
            pitch_motor->Enable();
        if (!yaw_motor->IsEnable())
            yaw_motor->Enable();

        gimbal->SetAbsTarget(0, 0);
        gimbal->Update();
        osDelay(GIMBAL_OS_DELAY);
        ++i;
    }

    // buzzer->SingTone(bsp::BuzzerNote::La6M);
    Buzzer_Sing(SingCaliStart);
    reset_yaw();
    ahrs->Cailbrate();
    i = 0;
    while (!ahrs->IsCailbrated()) {
        gimbal->SetAbsTarget(0, 0);
        gimbal->Update();
        osDelay(1);
        ++i;
    }
    Buzzer_Sing(SingCaliDone);
    osDelay(100);
    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    pitch_curr = ahrs->INS_angle[2];
    yaw_curr = ahrs->INS_angle[0];
    //    pitch_curr = witimu->INS_angle[0];
    //    yaw_curr = wrap<float>(witimu->INS_angle[2]-yaw_offset, -PI, PI);
    float pitch_target = 0, yaw_target = 0;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
            continue;
        }
        if (!pitch_motor->IsEnable())
            pitch_motor->Enable();
        if (!yaw_motor->IsEnable())
            yaw_motor->Enable();

        pitch_curr = ahrs->INS_angle[2];
        yaw_curr = ahrs->INS_angle[0];
        //        pitch_curr = witimu->INS_angle[0];
        //        yaw_curr = wrap<float>(witimu->INS_angle[2]-yaw_offset, -PI, PI);
        //    if (dbus->swr == remote::UP) {
        //      gimbal->SetAbsTarget(0, 0);
        //      gimbal->Update();
        //      pitch_target = pitch_curr;
        //      yaw_target = yaw_curr;
        //      control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        //      osDelay(1);
        //      continue;
        //    }
        if (selftest.dbus) {
            if (dbus->ch2||dbus->ch3){
                yaw_ratio = dbus->ch2 / 18000.0 / 7.0;
                pitch_ratio = -dbus->ch3 / 18000.0 / 7.0;
            } else {
                yaw_ratio = dbus->mouse.x / 32767.0 * 7.5 / 7.0;
                pitch_ratio = dbus->mouse.y / 32767.0 * 7.5 / 7.0;
            }
        } else if (selftest.refereerc) {
            pitch_ratio = -refereerc->remote_control.mouse.y / 32767.0 * 7.5 / 7.0;
            yaw_ratio = -refereerc->remote_control.mouse.x / 32767.0 * 7.5 / 7.0;
        } else {
            pitch_ratio = 0;
            yaw_ratio = 0;
        }

        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = wrap<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        //        if (-0.005 < pitch_diff && pitch_diff < 0.005) {
        //            pitch_diff = 0;
        //        }

        switch (remote_mode) {
            case REMOTE_MODE_SPIN:
            case REMOTE_MODE_FOLLOW:
                gimbal->SetRelTarget(pitch_diff, yaw_diff);
                gimbal->UpdateIMU(pitch_curr, yaw_curr);
                break;
            case REMOTE_MODE_MANUAL:
                gimbal->SetRelTarget(pitch_diff, yaw_diff);
                gimbal->Update();
                break;
            default:
                kill_gimbal();
        }

        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal() {
    pitch_motor = new driver::Motor6020(can2, 0x20A, 0x2FE);
    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 4000,
        .ki = 5,
        .kd = 500,
        .max_out = 16384,
        .max_iout = 4000,
        .deadband = 0,                          // 死区
        .A = 1.5 * PI,                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    pitch_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                         driver::MotorCANBase::ABSOLUTE);
    yaw_motor = new driver::Motor6020(can1, 0x209, 0x2FE);
    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_motor_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 3 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 2000,
        .ki = 0,
        .kd = 100,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                            // 死区
        .A = 0.5 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 0.5 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,      // 输出滤波系数
        .derivative_filtering_coefficient = 0.1,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    yaw_motor->ReInitPID(yaw_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    yaw_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                       driver::MotorCANBase::ABSOLUTE);

    control::gimbal_t gimbal_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}
void kill_gimbal() {
    yaw_motor->Disable();
    pitch_motor->Disable();
    // steering_motor->SetOutput(0);
}