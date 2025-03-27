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

static driver::Motor3508* flywheel_left = nullptr;
static driver::Motor3508* flywheel_right = nullptr;

driver::Motor2006* steering_motor = nullptr;
driver::ServoMG995* MG995;

int shoot_17mm_num = 0;

control::ConstrainedPID::PID_Init_t flywheel_pid_init = {
 .kp = 100,
 .ki = 1,
 .kd = 0,
 .max_out = 1000,
 .max_iout = 4000,
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
    flywheel_left = new driver::Motor3508(can2, 0x202);
    flywheel_right = new driver::Motor3508(can2, 0x201);
    steering_motor = new driver::Motor2006(can2, 0x203);

    MG995 = new driver::ServoMG995(&MG995_htim, MG995_channel);

    flywheel_left->SetTransmissionRatio(19);
    flywheel_right->SetTransmissionRatio(19);
    steering_motor->SetTransmissionRatio(36);

    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->ReInitPID(steering_motor_theta_pid_init, driver::MotorCANBase::THETA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);
    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);

    flywheel_left->ReInitPID(flywheel_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_left->SetMode(driver::MotorCANBase::OMEGA);

    flywheel_right->ReInitPID(flywheel_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_right->SetMode(driver::MotorCANBase::OMEGA);


    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);
    MG995->SetOutput(90);

    if(CAR_DEBUG->RM_Debug_mode.shooter_debug_mode == Debug_true){
        print("SHOOT_INIT_OK\n");
    }
}

void shootTask(void* arg){
    UNUSED(arg);
    osDelay(1000);
    motor_Disable();

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

    motor_Enable();

    Ease* flywheel_speed_ease = new Ease(0, 0.3);
    while (true) {

        if (remote_mode != REMOTE_MODE_KILL) {
            switch (magazine_Mode) {

                case Magazine_MODE_ON:
                    MG995->SetOutput(0);
                    break;
                case Magazine_MODE_OFF:
                    MG995->SetOutput(90);
                    break;
                default:
                    break;
            }
        }

        while(remote_mode == REMOTE_MODE_KILL || !referee->game_robot_status.mains_power_shooter_output) {
            shoot_kill();
            osDelay(SHOOT_OS_DELAY);
        }

        switch (shoot_flywheel_mode) {
            case SHOOT_FRIC_MODE_PREPARED:
                motor_Enable();
                // 启动摩擦轮电机(旋转)
                // todo:测试目标摩擦轮转速用[12 * PI], 最后改回[16 * PI]比赛使用
                flywheel_speed_ease->SetTarget(10 * PI);  // 摩擦轮转速(转速控制)
                break;
            case SHOOT_FRIC_MODE_PREPARING:
                motor_Enable();
                // todo:同上
                flywheel_speed_ease->SetTarget(8 * PI);  // 摩擦轮转速(转速控制)
                // 检测摩擦轮是否就绪
                if(flywheel_speed_ease->IsAtTarget()) {
                    shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
                }
                break;
            case SHOOT_FRIC_MODE_STOP:
                // 关闭摩擦轮电机(停止旋转)
                flywheel_speed_ease->SetTarget(0);    // 摩擦轮转速(转速控制)
                shoot_load_mode = SHOOT_MODE_STOP;
            default:
                break;
        }

        flywheel_speed_ease->Calc();
        flywheel_left->SetTarget(flywheel_speed_ease->GetOutput());
        flywheel_right->SetTarget(-flywheel_speed_ease->GetOutput());

        uint8_t game_progress = referee->game_status.game_progress;
        uint16_t game_progress_shifted = game_progress >> 3;
        uint8_t game_progress_bit = game_progress_shifted & 0x01;
        if (game_progress_bit == 4) {
            uint16_t bullet_remaining_num_17mm = referee->bullet_remaining.bullet_remaining_num_17mm;
            if (bullet_remaining_num_17mm == 0) {
                print("not have bullet!\n");
                steering_motor->Hold();
                continue;
            }
        }

        float bullet_speed = referee->shoot_data.bullet_speed;
        if (bullet_speed >= 23.0f) {
            osDelay(SHOOT_OS_DELAY);
            bullet_speed = 0;
        }

        int heat_limit = referee->game_robot_status.shooter_heat_limit;
        int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
        const int shooter_heat_threashold = 10;
        if (heat_buffer > heat_limit - shooter_heat_threashold) {
            // 临时解决方案
            steering_motor->Hold();
            print("overheat!\n");
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        UNUSED(heat_limit);
        UNUSED(heat_buffer);
        UNUSED(bullet_speed);


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

void motor_Enable() {
    steering_motor->Enable();
    flywheel_left->Enable();
    flywheel_right->Enable();
}

void motor_Disable() {
    // steering_motor->Disable();
    flywheel_left->Disable();
    flywheel_right->Disable();
}

void shoot_kill() {
    steering_motor->Hold(true);
    flywheel_left->Hold(true);
    flywheel_right->Hold(true);
    motor_Disable();
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

    switch (magazine_Mode) {
        case Magazine_MODE_ON:
            strcpy(s, "MODE_ON");
        break;
        case Magazine_MODE_OFF:
            strcpy(s, "MODE_OFF");
        break;
        default:
            break;
    }
    print("Magazine Mode:%s\r\n", s);
}