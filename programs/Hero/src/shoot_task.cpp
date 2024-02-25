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

driver::Motor3508* steering_motor = nullptr;

void jam_callback(void* args) {
    driver::Motor3508* motor = static_cast<driver::Motor3508*>(args);
    float target = motor->GetTarget();
    if (target > motor->GetOutputShaftTheta()) {
        float prev_target = motor->GetTarget() - 2 * PI / 5;
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
        //                if (referee->bullet_remaining.bullet_remaining_num_17mm == 0){
        //                    //没子弹了
        //                    shoot_flywheel_offset = -200;
        //                    flywheel_left->SetOutput(ramp_1.Calc(shoot_flywheel_offset));
        //                    flywheel_right->SetOutput(ramp_2.Calc(shoot_flywheel_offset));
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                    kill_shoot();
        //                    osDelay(SHOOT_OS_DELAY);
        //                    continue;
        //                }
        //         检测开关状态，向上来回打即启动拔弹
        //        if (can_shoot_click) {
        //            if (dbus->keyboard.bit.CTRL == 1) {
        //                if (shoot_state == 0) {
        //                    shoot_state = 1;
        //                } else {
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                }
        //            }
        //            if (shoot_state != 0) {
        //                if (dbus->mouse.l == 1) {
        //                    shoot_state_2 = 2;
        //                } else if (dbus->mouse.l == 0) {
        //                    shoot_state_2 = 0;
        //                }
        //            }
        //        }
        //        if (dbus->keyboard.bit.CTRL == 0) {
        //            can_shoot_click = true;
        //        } else {
        //            can_shoot_click = false;
        //        }
        //        if (dbus->swl == remote::UP) {
        //            if (last_state == remote::MID)
        //                last_state = remote::UP;
        //        } else if (dbus->swl == remote::MID) {
        //            if (last_state == remote::UP) {
        //                last_state = remote::MID;
        //                if (shoot_state == 0) {
        //                    shoot_state = 1;
        //                } else {
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                }
        //            }
        //        }
        //        switch (shoot_state) {
        //            case 0:
        //                shoot_flywheel_offset = -200;
        //
        //                break;
        //            case 1:
        //            case 2:
        //                shoot_flywheel_offset = 200;
        //                if (servo_back == 0) {
        //                    load_servo->SetTarget(load_servo->GetTheta() - 2 * PI / 32, true);
        //                    servo_back = 1;
        //                }
        //                break;
        //        }
        //        if (shoot_state == 1 && ramp_1.Get() == ramp_1.GetMax() &&
        //            ramp_2.Get() == ramp_2.GetMax()) {
        //            shoot_state = 2;
        //        }
        switch (shoot_fric_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
                flywheel_left->SetTarget(600.0f / 6 * 2 * PI);
                flywheel_right->SetTarget(600.0f / 6 * 2 * PI);
                shoot_fric_mode = SHOOT_FRIC_MODE_PREPARED;
                shoot_mode = SHOOT_MODE_PREPARED;
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
        if (shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED) {
            switch (shoot_mode) {
                case SHOOT_MODE_PREPARING:
                case SHOOT_MODE_PREPARED:
                    // 准备就绪，未发射状态
                    // 如果检测到未上膛（刚发射一枚子弹），则回到准备模式
                    //                    if (!steering_motor->IsHolding()) {
                    //                        steering_motor->SetTarget(steering_motor->GetTheta());
                    //                    }
                    break;
                case SHOOT_MODE_SINGLE:
                    // 发射一枚子弹
                    if (last_shoot_mode != SHOOT_MODE_SINGLE) {
                        if (steering_motor->IsHolding()) {
                            steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 5);
                        }
                        shoot_mode = SHOOT_MODE_PREPARED;
                    }
                    break;
                case SHOOT_MODE_STOP:
                    // 停止发射
                    break;
                default:
                    break;
            }
        }
        last_shoot_mode = shoot_mode;
        //        // 启动拔弹电机后的操作
        //        if (shoot_state == 2) {
        //            // 检测是否已装填子弹
        //
        //            // 检测是否需要发射子弹
        //
        //                if (dbus->swl == remote::DOWN) {
        //                    if (last_state_2 == remote::MID) {
        //                        last_state_2 = remote::DOWN;
        //                        if (shoot_state_2 == 0) {
        //                            shoot_state_2 = 1;
        //                        }
        //                        shoot_time_count = 0;
        //                    }
        //                    shoot_time_count++;
        //                    if (shoot_time_count > 1000 / SHOOT_OS_DELAY) {
        //                        shoot_state_2 = 2;
        //                    }
        //                } else if (dbus->swl == remote::MID) {
        //                    if (last_state_2 == remote::DOWN) {
        //                        last_state_2 = remote::MID;
        //                    }
        //                    shoot_state_2 = 0;
        //                }
        //            // 发射子弹
        //            if (shoot_state_2 == 1) {
        //                // 检测是否已经发射完毕
        //                if (last_shoot_key == 0 && shoot_state_key == 1) {
        //                    last_shoot_key = 1;
        //                } else if (last_shoot_key == 1 && shoot_state_key == 0) {
        //                    last_shoot_key = 0;
        //                    shoot_state_2 = 0;
        //                }
        //                // 如果发射未完成，则需要发射子弹
        //                if (shoot_state_2 == 1) {
        //                    load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
        //                } else {
        //                    if (!load_servo->Holding()) {
        //                        load_servo->SetTarget(load_servo->GetTheta(), true);
        //                    }
        //                }
        //            } else if (shoot_state_2 == 2) {
        //                // 连续发射
        //                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
        //            } else if (shoot_state_key == 1) {
        //                // 不需要发射子弹，但是未装弹完毕，则需要装填子弹
        //                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
        //            } else {
        //                // 不需要发射子弹，且装弹完毕，则需要锁定拔弹电机
        //                if (!load_servo->Holding()) {
        //                    load_servo->SetTarget(load_servo->GetTheta(), true);
        //                }
        //            }
        //        }
        // 计算输出，由于拔弹电机的输出系统由云台托管，不需要再次处理can的传输

        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::Motor3508(can2, 0x201);
    flywheel_right = new driver::Motor3508(can2, 0x202);
    flywheel_left->SetTransmissionRatio(1);
    flywheel_right->SetTransmissionRatio(1);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    flywheel_left->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_right->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_left->SetMode(driver::MotorCANBase::OMEGA);
    flywheel_right->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::INVERTED);

    steering_motor = new driver::Motor3508(can1, 0x202);

    steering_motor->SetTransmissionRatio(19);
    control::ConstrainedPID::PID_Init_t steering_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 2 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波

    };
    steering_motor->ReInitPID(steering_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t steering_omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::ErrorHandle,            // 错误处理
    };
    steering_motor->ReInitPID(steering_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);

    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->Disable();
    flywheel_left->Disable();
    flywheel_right->Disable();
}