/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

static driver::Motor3508* flywheel_left = nullptr;
static driver::Motor3508* flywheel_right = nullptr;
static float* pid1_param = nullptr;
static float* pid2_param = nullptr;
static driver::FlyWheelMotor* flywheel1 = nullptr;
static driver::FlyWheelMotor* flywheel2 = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

driver::ServoMotor* load_servo = nullptr;

void jam_callback(driver::ServoMotor* servo, const driver::servo_jam_t data) {
    UNUSED(data);
    float servo_target = servo->GetTarget();
    if (servo_target > servo->GetTheta()) {
        float prev_target = servo->GetTarget() - 2 * PI / 8;
        servo->SetTarget(prev_target, true);
    }
}

osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    driver::MotorCANBase* motors[] = {flywheel_left, flywheel_right, steering_motor};
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

    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->CalcOutput();

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
                flywheel1->SetSpeed(200.0f / 6 * 5 * PI);
                flywheel2->SetSpeed(200.0f / 6 * 5 * PI);
                shoot_fric_mode = SHOOT_FRIC_MODE_PREPARED;
                shoot_mode = SHOOT_MODE_PREPARED;
                break;
            case SHOOT_FRIC_MODE_PREPARED:
                break;
            case SHOOT_FRIC_MODE_STOP:
                flywheel1->SetSpeed(0);
                flywheel2->SetSpeed(0);
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
                    if (!load_servo->Holding()) {
                        load_servo->SetTarget(load_servo->GetTheta(), false);
                    }
                    break;
                case SHOOT_MODE_SINGLE:
                    // 发射一枚子弹
                    if (last_shoot_mode != SHOOT_MODE_SINGLE) {
                        load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, true);
                        shoot_mode = SHOOT_MODE_PREPARED;
                    }
                    break;
                case SHOOT_MODE_BURST:
                    // 连发子弹
                    load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
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
        load_servo->CalcOutput();
        flywheel1->CalcOutput();
        flywheel2->CalcOutput();
        driver::MotorCANBase::TransmitOutput(motors, 3);
        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::Motor3508(can2, 0x201);
    flywheel_right = new driver::Motor3508(can2, 0x202);
    pid1_param = new float[3]{150, 1, 0.15};
    pid2_param = new float[3]{150, 1, 0.15};
    driver::flywheel_t flywheel1_data = {
        .motor = flywheel_left,
        .max_speed = 400 * PI,
        .omega_pid_param = pid1_param,
        .is_inverted = false,
    };
    driver::flywheel_t flywheel2_data = {
        .motor = flywheel_right,
        .max_speed = 400 * PI,
        .omega_pid_param = pid2_param,
        .is_inverted = true,
    };
    flywheel1 = new driver::FlyWheelMotor(flywheel1_data);
    flywheel2 = new driver::FlyWheelMotor(flywheel2_data);
    flywheel1->SetSpeed(0);
    flywheel2->SetSpeed(0);

    steering_motor = new driver::Motor2006(can2, 0x203);

    driver::servo_t servo_data;
    servo_data.motor = steering_motor;
    servo_data.max_speed = 2.5 * PI;
    servo_data.max_acceleration = 16 * PI;
    servo_data.transmission_ratio = M2006P36_RATIO;
    servo_data.omega_pid_param = new float[3]{6000, 80, 0.3};
    servo_data.max_iout = 4000;
    servo_data.max_out = 10000;
    servo_data.hold_pid_param = new float[3]{150, 2, 0.01};
    servo_data.hold_max_iout = 2000;
    servo_data.hold_max_out = 10000;

    load_servo = new driver::ServoMotor(servo_data);
    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->RegisterJamCallback(jam_callback, 0.6);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
    driver::MotorCANBase* motors[] = {flywheel_left, flywheel_right, steering_motor};
    driver::MotorCANBase::TransmitOutput(motors, 3);
}