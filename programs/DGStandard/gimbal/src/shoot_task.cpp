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

static driver::MotorPWMBase* flywheel_left = nullptr;
static driver::MotorPWMBase* flywheel_right = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

driver::ServoMotor* load_servo = nullptr;

bsp::GPIO* shoot_key = nullptr;

// 堵转回调函数
void jam_callback(driver::ServoMotor* servo, const driver::servo_jam_t data) {
    UNUSED(data);
    // 反向旋转
    float servo_target = servo->GetTarget();
    if (servo_target > servo->GetTheta()) {
        float prev_target = servo->GetTarget() - 2 * PI / 8;
        servo->SetTarget(prev_target, true);
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
    while (ahrs->IsCailbrated()) {
        osDelay(SHOOT_OS_DELAY);
    }

    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->CalcOutput();

    Ease* flywheel_speed_ease = new Ease(0, 0.3);

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            // 死了
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
            shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
            flywheel_speed_ease->SetTarget(400);
        } else {
            flywheel_speed_ease->SetTarget(0);
        }
        flywheel_speed_ease->Calc();
        flywheel_left->SetOutput(flywheel_speed_ease->GetOutput());
        flywheel_right->SetOutput(flywheel_speed_ease->GetOutput());

        // 检测摩擦轮是否就绪
        // 由shoot_task切换到SHOOT_FRIC_MODE_PREPARED
        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING && flywheel_speed_ease->IsAtTarget()) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
        }
        if (shoot_flywheel_mode != SHOOT_FRIC_MODE_PREPARED) {
            load_servo->Hold();
            load_servo->CalcOutput();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        /* 摩擦轮就绪后执行以下部分 */

        if (shoot_load_mode == SHOOT_MODE_STOP) {
            load_servo->Hold(true);
            // todo: 这里为什么要是true才能停下？target_angel为什么会超过范围？
        }
        if (shoot_load_mode == SHOOT_MODE_IDLE) {
            uint8_t loaded = shoot_key->Read();
            if (loaded) {
                load_servo->Hold();
            } else {
                // 没有准备就绪，则旋转拔弹电机
                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
            }
        }
        if (shoot_load_mode == SHOOT_MODE_SINGLE) {
            load_servo->SetTarget(load_servo->GetTheta() + 2 * PI / 8, true);
            shoot_load_mode = SHOOT_MODE_IDLE;
        }
        if (shoot_load_mode == SHOOT_MODE_BURST) {
            load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, true);
        }

        // 计算输出
        load_servo->CalcOutput();
        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::MotorPWMBase(&htim1, 1, 1000000, 500, 1000);
    flywheel_right = new driver::MotorPWMBase(&htim1, 4, 1000000, 500, 1000);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);

    steering_motor = new driver::Motor2006(can1, 0x207);

    driver::servo_t servo_data = {
        .motor = steering_motor,
        .max_speed = 2.5 * PI,
        .max_acceleration = 16 * PI,
        .transmission_ratio = M2006P36_RATIO,
        .omega_pid_param = new float[3]{6000, 80, 0.3},
        .max_iout = 4000,
        .max_out = 10000,
        .hold_pid_param = nullptr,
        .hold_max_iout = 2000,
        .hold_max_out = 10000,
    };

    load_servo = new driver::ServoMotor(servo_data);
    load_servo->RegisterJamCallback(jam_callback, 0.6);
    load_servo->Hold(true);

    shoot_key = new bsp::GPIO(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
}