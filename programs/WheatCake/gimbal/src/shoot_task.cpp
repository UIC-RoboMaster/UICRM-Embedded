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

driver::Motor2006* steering_motor = nullptr;

const float singleShotDivider = 3.5f;

bool jam_notify_flags = false;

void jam_callback(void* args) {
    driver::Motor2006* motor = static_cast<driver::Motor2006*>(args);
    jam_notify_flags = true;
    float target = motor->GetTarget();
    if (target > motor->GetOutputShaftTheta()) {
        float prev_target = motor->GetTarget() - 2 * PI / singleShotDivider;
        motor->SetTarget(prev_target);
    } else {
        float prev_target = motor->GetTarget() + 2 * PI / singleShotDivider;
        motor->SetTarget(prev_target);
    }
}

osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    // 启动等待
    osDelay(1000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }
    // 等待IMU初始化
    while (!imu->DataReady() || !imu->CaliDone()) {
        osDelay(SHOOT_OS_DELAY);
    }
    //    int last_state = remote::MID;
    //    int last_state_2 = remote::MID;
    //    uint8_t shoot_state = 0;
    //    int shoot_flywheel_offset = 0;
    //    uint8_t shoot_state_2 = 0;
    //    uint8_t last_shoot_key = 0;
    //    uint16_t shoot_time_count = 0;
    //    uint8_t servo_back = 0;
    //    bool can_shoot_click = false;

    ShootMode last_shoot_mode = SHOOT_MODE_STOP;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            // 死了
            //            shoot_flywheel_offset = -5000;

            //            shoot_state = 0;
            //            shoot_state_2 = 0;
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        if (!steering_motor->IsEnable()) {
            steering_motor->Enable();
        }
        if (!flywheel_left->IsEnable()) {
            flywheel_left->Enable();
        }
        if (!flywheel_right->IsEnable()) {
            flywheel_right->Enable();
        }

        switch (shoot_flywheel_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
                flywheel_left->SetTarget(120.0f * 2 * PI);
                flywheel_right->SetTarget(120.0f * 2 * PI);
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
                shoot_load_mode = SHOOT_MODE_PREPARED;
                break;
            case SHOOT_FRIC_MODE_PREPARED:
                break;
            case SHOOT_FRIC_MODE_STOP:
                flywheel_left->SetTarget(0);
                flywheel_right->SetTarget(0);
                // laser->SetOutput(0);
                break;
            default:
                //                shoot_flywheel_offset = -1000;
                // laser->SetOutput(0);
                break;
        }
        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
            switch (shoot_load_mode) {
                case SHOOT_MODE_PREPARING:
                case SHOOT_MODE_PREPARED:
                    // 准备就绪，未发射状态
                    // 如果检测到未上膛（刚发射一枚子弹），则回到准备模式
                    //                    if (!steering_motor->IsHolding()) {
                    //                        steering_motor->SetTarget(steering_motor->GetTheta());
                    //                    }
                    if (last_shoot_mode == SHOOT_MODE_BURST)
                        steering_motor->Hold(true);
                    break;
                case SHOOT_MODE_SINGLE:
                    // 发射一枚子弹
                    if (last_shoot_mode != SHOOT_MODE_SINGLE) {
                        if (steering_motor->IsHolding()) {
                            steering_motor->SetTarget(
                                steering_motor->GetTarget() + 2 * PI / singleShotDivider, true);
                        }
                        shoot_load_mode = SHOOT_MODE_PREPARED;
                    }
                    break;
                case SHOOT_MODE_BURST:
                    steering_motor->SetTarget(
                        steering_motor->GetTarget() + 2 * PI / singleShotDivider, true);
                    break;
                case SHOOT_MODE_STOP:
                    // 停止发射
                    steering_motor->Hold(true);
                    break;
                default:
                    break;
            }
        }
        last_shoot_mode = shoot_load_mode;

        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::Motor3508(can2, 0x201);
    flywheel_right = new driver::Motor3508(can2, 0x202);
    flywheel_left->SetTransmissionRatio(1);
    flywheel_right->SetTransmissionRatio(1);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 150,
        .ki = 0.03,
        .kd = 1,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                                          // 死区
        .A = 3 * PI,                                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    flywheel_left->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    flywheel_right->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    flywheel_left->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                           driver::MotorCANBase::REVERSE_MOTOR_OPERATE);
    flywheel_right->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);

    steering_motor = new driver::Motor2006(can2, 0x203);

    steering_motor->SetTransmissionRatio(19);
    control::ConstrainedPID::PID_Init_t steering_theta_pid_init = {
        .kp = 30,
        .ki = 0,
        .kd = 300,
        .max_out = 4 * PI,
        .max_iout = 0.25 * PI,
        .deadband = 0,                                        // 死区
        .A = 0,                                               // 变速积分所能达到的最大值为A+B
        .B = 0,                                               // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                  // 输出滤波系数
        .derivative_filtering_coefficient = 0,                // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |     // 积分限幅
                control::ConstrainedPID::OutputFilter |       // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral  // 梯形积分
    };
    steering_motor->ReInitPID(steering_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t steering_omega_pid_init = {
        .kp = 800,
        .ki = 0,
        .kd = 5000,
        .max_out = 10000,
        .max_iout = 0,
        .deadband = 0,                                        // 死区
        .A = 2 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1.5 * PI,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                  // 输出滤波系数
        .derivative_filtering_coefficient = 0,                // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |     // 积分限幅
                control::ConstrainedPID::OutputFilter |       // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral  // 梯形积分
    };
    steering_motor->ReInitPID(steering_omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    steering_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                            driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    //    steering_motor->SetMode(driver::MotorCANBase::OMEGA);

    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->Disable();
    flywheel_left->Disable();
    flywheel_right->Disable();
}