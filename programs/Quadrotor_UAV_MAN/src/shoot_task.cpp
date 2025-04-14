// Copyright (c) 2023-2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

#include "shoot_task.h"
#include "MotorPWMBase.h"

#define LEFT_MOTOR_PWM_CHANNEL 1
#define RIGHT_MOTOR_PWM_CHANNEL 4
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50
#define BLHEIL_MOTOR_MIN_PULSE_WIDTH 1000

driver::Motor2006* steering_motor = nullptr;

static driver::MotorPWMBase* flyWheelL = nullptr;
static driver::MotorPWMBase* flyWheelR = nullptr;

Ease* flyWheelEase = nullptr;

int shoot_17mm_num = 0;

control::ConstrainedPID::PID_Init_t steering_motor_theta_pid_init = {
    .kp = 30,
    .ki = 0,
    .kd = 0,
    .max_out = 5 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};
control::ConstrainedPID::PID_Init_t steering_motor_omega_pid_init = {
    .kp = 200,
    .ki = 0,
    .kd = 10000,
    .max_out = 10000,
    .max_iout = 4000,
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

void init_shoot(){
    init_pwm();

    steering_motor = new driver::Motor2006(can1, 0x201);
    steering_motor->SetTransmissionRatio(36);

    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->ReInitPID(steering_motor_theta_pid_init, driver::MotorCANBase::THETA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);
    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);

}

void init_pwm() {
    flyWheelL = new driver::MotorPWMBase(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, BLHEIL_MOTOR_MIN_PULSE_WIDTH);
    flyWheelR = new driver::MotorPWMBase(&htim1, RIGHT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, BLHEIL_MOTOR_MIN_PULSE_WIDTH);

    flyWheelL->Disable();
    flyWheelR->Disable();

    // 设置PWM信号为1000 + 1000 = 2000us（最高点）
    flyWheelL->SetOutput(1000);
    flyWheelR->SetOutput(1000);
    print("PWM set to 2000us for calibration.\r\n");
    HAL_Delay(2000);  // 等待电调进入校准模式

    // 设置PWM信号为1000 + 0 = 1000us（最低点）
    flyWheelL->SetOutput(0);
    flyWheelR->SetOutput(0);
    print("PWM set to 1000us for calibration.\r\n");
    HAL_Delay(2000);  // 等待电调完成校准
}

void shootTask(void* arg){
    UNUSED(arg);
    osDelay(1000);

    while (protect_wraning_flag) {
        osDelay(PROTECT_OS_DELAY);
    }

    while(remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }

    // 等待IMU初始化
    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    const float flyWheelMaxTarget = 450.0;
    flyWheelEase = new Ease(0, 1);
    while (true) {

        while(remote_mode == REMOTE_MODE_KILL) {
            shoot_kill();
            osDelay(SHOOT_OS_DELAY);
        }

        if (!flyWheelL->isEnable())
            flyWheelL->Enable();
        if (!flyWheelR->isEnable())
            flyWheelR->Enable();
        if (!steering_motor->IsEnable())
            steering_motor->Enable();

//        if(!flyWheelEase->IsAtTarget()) {
//            shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARING;
//        }
        switch (shoot_flywheel_mode) {
            case SHOOT_FRIC_MODE_PREPARED:
                // 启动摩擦轮电机(旋转)
                break;
            case SHOOT_FRIC_MODE_PREPARING: {
                flyWheelEase->SetTarget(flyWheelMaxTarget);
                auto preStageTar = static_cast<int16_t>(flyWheelEase->Calc());
                flyWheelL->SetOutput(preStageTar);
                flyWheelR->SetOutput(preStageTar);
                // 检测摩擦轮是否就绪
                if (flyWheelEase->IsAtTarget()) {
                    shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
                }
                break;
            }
            case SHOOT_FRIC_MODE_STOP: {
                // 关闭摩擦轮电机(停止旋转)
                flyWheelEase->SetTarget(0);
                auto stopStageTar = static_cast<int16_t>(flyWheelEase->Calc());
                flyWheelL->SetOutput(stopStageTar);  // 摩擦轮转速(转速控制)
                flyWheelR->SetOutput(stopStageTar);
                break;
            }
            case SHOOT_FRIC_MODE_DISABLE:
                flyWheelL->Disable();
                flyWheelR->Disable();
                break;
            default:
                break;
        }

        // todo: 裁判系统暂时关闭
        // uint8_t game_progress = referee->game_status.game_progress;
        // uint16_t game_progress_shifted = game_progress >> 3;
        // uint8_t game_progress_bit = game_progress_shifted & 0x01;
        // if (game_progress_bit == 4) {
        //     uint16_t bullet_remaining_num_17mm = referee->bullet_remaining.bullet_remaining_num_17mm;
        //     if (bullet_remaining_num_17mm == 0) {
        //         print("not have bullet!\n");
        //         steering_motor->Hold();
        //         continue;
        //     }
        // }

        // float bullet_speed = referee->shoot_data.bullet_speed;
        // if (bullet_speed >= 23.0f) {
        //     osDelay(SHOOT_OS_DELAY);
        //     bullet_speed = 0;
        // }
        //
        // int heat_limit = referee->game_robot_status.shooter_heat_limit;
        // int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
        // const int shooter_heat_threashold = 10;
        // if (heat_buffer > heat_limit - shooter_heat_threashold) {
        //     // 临时解决方案
        //     steering_motor->Hold();
        //     print("overheat!\n");
        //     osDelay(SHOOT_OS_DELAY);
        //     continue;
        // }
        // UNUSED(heat_limit);
        // UNUSED(heat_buffer);
        // UNUSED(bullet_speed);


        switch (shoot_load_mode){
            case SHOOT_MODE_STOP:
                steering_motor->Hold(true);
                break;
            case SHOOT_MODE_IDLE:
                break;
            case SHOOT_MODE_SINGLE:
                Shoot_Mode_SINGLE(steering_motor->GetTarget() + 2 * PI / singleShotDivider, 2);
                break;
            case SHOOT_MODE_BURST:
                Shoot_Mode_BURST(steering_motor->GetTarget() + 2 * PI / 8);
                break;
            case SHOOT_MODE_UNLOAD:
                Shoot_Mode_UNLOAD(-steering_motor->GetTarget() - PI / 8);

            default:
                break;
        }

        osDelay(SHOOT_OS_DELAY);
    }
}

void Shoot_Mode_SINGLE(float target, int num) {
    int last_steering_target = steering_motor->GetTheta();
    steering_motor->SetTarget(target, true);
    shoot_17mm_num += num;
    while (true){
        if(steering_motor->GetTheta() != last_steering_target){
            break;
        } else {
            osDelay(SHOOT_SINGLE_OS_DELAY);
        }
    }
    shoot_load_mode = SHOOT_MODE_IDLE;
}

void Shoot_Mode_BURST(float target, int num) {
    steering_motor->SetTarget(target, true);
    shoot_17mm_num += num;
    // shoot_load_mode = SHOOT_MODE_IDLE;
}

void Shoot_Mode_UNLOAD(float target) {
    int last_steering_target = steering_motor->GetTheta();
    steering_motor->SetTarget(target, true);
    while (true){
        if(steering_motor->GetTheta() != last_steering_target){
            break;
        } else {
            osDelay(SHOOT_SINGLE_OS_DELAY);
        }
    }
    shoot_load_mode = SHOOT_MODE_IDLE;
}


void shoot_kill() {
    steering_motor->Hold(true);
    flyWheelL->Disable();
    flyWheelR->Disable();
}

void shoot_remote_mode(){
    char s[20];
    switch (shoot_flywheel_mode) {
        case SHOOT_FRIC_MODE_PREPARING:
            strcpy(s, "PREPAREING");
        break;
        case SHOOT_FRIC_MODE_STOP:
            strcpy(s, "STOP");
        break;
        case SHOOT_FRIC_MODE_PREPARED:
            strcpy(s, "PREPARED");
        break;
        case SHOOT_FRIC_MODE_DISABLE:
            strcpy(s, "DISABLE");
        break;
        default:
            strcpy(s, "UNKNOWN");
        break;
    }

    print("Shoot Fric Mode:%s\r\n", s);
    switch (shoot_load_mode) {
        case SHOOT_MODE_STOP:
            strcpy(s, "STOP");
        break;
        case SHOOT_MODE_IDLE:
            strcpy(s, "IDLE");
        break;
        case SHOOT_MODE_DISABLE:
            strcpy(s, "DISABLE");
        break;
        case SHOOT_MODE_UNLOAD:
            strcpy(s, "UNLOAD");
        break;
        case SHOOT_MODE_SINGLE:
            strcpy(s, "SINGLE");
        break;
        case SHOOT_MODE_BURST:
            strcpy(s, "BURST");
        break;
        case SHOOT_MODE_RELOADING:
            strcpy(s, "RELOADING");
        break;
        default:
            strcpy(s, "UNKNOWN");
        break;
    }
    print("Shoot Lode Mode:%s\r\n", s);

}