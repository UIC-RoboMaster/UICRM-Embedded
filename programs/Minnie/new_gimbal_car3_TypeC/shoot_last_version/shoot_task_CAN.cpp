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

#include "shoot_task_CAM.h"

//static driver::MotorPWMBase* flywheel_left = nullptr;
//static driver::MotorPWMBase* flywheel_right = nullptr;

static driver::Motor3508* flywheel_left = nullptr;
static driver::Motor3508* flywheel_right = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

driver::ServoMG995* MG995 = nullptr;

bool jam_notify_flags = false;

int shooter_17mm_num = 0;

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

bool steering_SetTarget(driver::MotorCANBase* motor, float SetTarget ,float threshold, bool locked_rotor = true, bool ABS = false){
    motor->Enable();
    bool Steering_Mode = false;
    bool motor_flag = false;
    int locked_rotor_ticks = 0;

    if (motor->GetOutomega() >= -0.5f && motor->GetOutomega() <= 0.5f) {
        motor_flag = false;
    }else {
        motor_flag = true;
    }

    motor->SetTarget(SetTarget);
    osDelay(500);
    while(locked_rotor) {

        if (motor_flag == true) {
            print("%f\n", SetTarget - motor->GetOutputShaftOmega());
            if(SetTarget - motor->GetOutputShaftOmega() >= (threshold*36*SetTarget)/1000) {
                osDelay(10);
                locked_rotor_ticks++;
            }else {
                break;
            }
        } else {
            break;
        }

        if (locked_rotor_ticks >= 3) {
            Steering_Mode = true;
            motor->SetTarget(0);
            if (ABS == true) {
                motor->Disable();
            }
            print("Locked rotor\n");
            osDelay(20);
            break;
        }
    }
    return Steering_Mode;
}

void steering_unload(bool unload_sign) {
    if (unload_sign == true) {
        print("Steering Mode\n");
        float last_theta = steering_motor->GetTheta();
        while (true) {
            if (steering_motor->GetTheta() != last_theta) {
                // steering_motor->SetTarget(0);
                break;
            }else {
                steering_motor->SetTarget(-PI/2);
                osDelay(500);
                steering_motor->SetTarget(PI/4);
                osDelay(500);
                steering_motor->SetTarget(-PI/2);
                osDelay(500);
            }
        }
    }
}

void steering_stop(){
    steering_motor->SetTarget(0);
    steering_motor->Hold(true);
}

void Steering_SINGLE(){
    int last_steering_target = steering_motor->GetTheta();
    bool Shoot_Steering_Mode = steering_SetTarget(steering_motor, steering_motor->GetTarget() + 2 * PI / 8, 0.65, false, false);
    steering_unload(Shoot_Steering_Mode);
    shooter_17mm_num ++;
    while (true){
        if(steering_motor->GetTheta() != last_steering_target){
            break;
        } else {
            osDelay(SHOOT_SINGLE_OS_DELAY);
        }
    }
    steering_motor->Hold(true);
    shoot_load_mode = SHOOT_MODE_IDLE;
}

void Stering_BURST(){
    int last_steering_target = steering_motor->GetTheta();
    bool Shoot_Steering_Mode = steering_SetTarget(steering_motor, steering_motor->GetTarget() + 2 * PI / 8, 0.65, false, false);
    steering_unload(Shoot_Steering_Mode);
    shooter_17mm_num ++;
    while (true){
        if(steering_motor->GetTheta() != last_steering_target){
            break;
        } else {
            osDelay(SHOOT_SINGLE_OS_DELAY);
        }
    }
    steering_motor->Hold(true);
    shoot_load_mode = SHOOT_MODE_IDLE;
}

void Streeing_Reloading(){
    steering_stop();
}

osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    // 启动等待作为锁定转子时间，并将htim_sign设置为true。
    osDelay(1000);

    while (protect_wraning_flag) {
        osDelay(PROTECT_OS_DELAY);
    }

    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }

    // 等待IMU初始化
    while (ahrs->IsCailbrated()) {
        osDelay(SHOOT_OS_DELAY);
    }

    // MG995->SetOutput(0);

    steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());
    Ease* flywheel_speed_ease = new Ease(0, 0.3);

    while (true) {

        if (remote_mode == REMOTE_MODE_KILL ||
            !referee->game_robot_status.mains_power_shooter_output) {
            // 死了
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
            shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
            // 启动摩擦轮电机(旋转)
            // todo:测试目标摩擦轮转速用[2 * PI], 最后改回[60 * 2 * PI]比赛使用
            flywheel_speed_ease->SetTarget(10 * PI);  // 摩擦轮转速(转速控制)
        } else {
            // 关闭摩擦轮电机(停止旋转)
            flywheel_speed_ease->SetTarget(0);    // 摩擦轮转速(转速控制)
            shoot_load_mode = SHOOT_MODE_STOP;
        }

        flywheel_speed_ease->Calc();
        flywheel_left->SetTarget(flywheel_speed_ease->GetOutput());
        flywheel_right->SetTarget(-flywheel_speed_ease->GetOutput());

        // 检测摩擦轮是否就绪
        // 由shoot_task切换到SHOOT_FRIC_MODE_PREPARED
        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING && flywheel_speed_ease->IsAtTarget()) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
        }

        /* 摩擦轮就绪后执行以下部分 */

        switch (shoot_load_mode) {
            case SHOOT_MODE_DISABLE:
                break;
            case SHOOT_MODE_UNLOAD:
                break;
            case SHOOT_MODE_STOP:
                steering_stop();
                break;
            case SHOOT_MODE_IDLE:
                steering_stop();
                break;
            case SHOOT_MODE_SINGLE:
                Steering_SINGLE();
                break;
            case SHOOT_MODE_BURST:
                Stering_BURST();
                break;
            case SHOOT_MODE_RELOADING:
                Streeing_Reloading();
                break;
            default:
                break;
        }
        /* 弹仓控制 */

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
        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {

    flywheel_left = new driver::Motor3508(can2, 0x202);
    flywheel_right = new driver::Motor3508(can2, 0x201);
    steering_motor = new driver::Motor2006(can2, 0x203);

    MG995 = new driver::ServoMG995(&MG995_htim, MG995_channel);

    flywheel_left->SetTransmissionRatio(19);
    flywheel_right->SetTransmissionRatio(19);
    steering_motor->SetTransmissionRatio(36);
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
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
        .kp = 1000,
        .ki = 0,
        .kd = 0,
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
    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->ReInitPID(theta_pid_init, driver::MotorCANBase::THETA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);

    flywheel_left->ReInitPID(flywheel_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_left->SetMode(driver::MotorCANBase::OMEGA);
    flywheel_left->RegisterErrorCallback(jam_callback, flywheel_left);

    flywheel_right->ReInitPID(flywheel_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_right->SetMode(driver::MotorCANBase::OMEGA);
    flywheel_right->RegisterErrorCallback(jam_callback, flywheel_right);

    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);

    if(CAR_DEBUG->RM_Debug_mode.shooter_debug_mode == Debug_true){
        print("SHOOT_INIT_OK\n");
    }
}

void kill_shoot() {
    shooter_17mm_num = 0;
    steering_motor->Hold(true);
    // flywheel_left->SetTarget(0);
    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);
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