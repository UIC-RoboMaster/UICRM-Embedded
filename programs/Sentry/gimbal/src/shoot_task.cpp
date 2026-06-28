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
        check_kill_shoot();

        switch (shoot_flywheel_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
                flywheel_left->SetTarget(250.0f * PI);
                flywheel_right->SetTarget(250.0f * PI);
                shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
                shoot_load_mode = SHOOT_MODE_IDLE;
                break;
            case SHOOT_FRIC_MODE_PREPARED:
                flywheel_left->SetTarget(250.0f * PI);
                flywheel_right->SetTarget(250.0f * PI);
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
            int heat_limit = referee->game_robot_status.shooter_heat_limit;
            int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
            const int shooter_heat_threashold = 25;
            if (heat_buffer > heat_limit - shooter_heat_threashold) {
                // 临时解决方案
                steering_motor->Hold();
                last_shoot_mode = shoot_load_mode;
                osDelay(SHOOT_OS_DELAY);
                continue;
            }
            switch (shoot_load_mode) {
                case SHOOT_MODE_IDLE:
                    // 准备就绪，未发射状态
                    // 如果检测到未上膛（刚发射一枚子弹），则回到准备模式
                    if (!steering_motor->IsHolding()) {
                        steering_motor->Hold();
                    }
                    break;
                case SHOOT_MODE_SINGLE:
                    // 发射一枚子弹
                    if (last_shoot_mode != SHOOT_MODE_SINGLE) {
                        steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, true);
                        shoot_load_mode = SHOOT_MODE_IDLE;
                    }
                    break;
                case SHOOT_MODE_BURST:
                    // 连发子弹
                    steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, false);
                    break;
                case SHOOT_MODE_STOP:
                    // 停止发射
                    break;
                default:
                    break;
            }
        }
        last_shoot_mode = shoot_load_mode;

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
    flywheel_left->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_right->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_left->SetMode(driver::MotorCANBase::OMEGA | driver::MotorCANBase::INVERTED);
    flywheel_right->SetMode(driver::MotorCANBase::OMEGA);

    steering_motor = new driver::Motor2006(can2, 0x207);

    steering_motor->SetTransmissionRatio(36);
    control::ConstrainedPID::PID_Init_t steering_motor_theta_pid_init = {
        .kp = 40,
        .ki = 0,
        .kd = 0,
        .max_out = 4 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    steering_motor->ReInitPID(steering_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t steering_motor_omega_pid_init = {
        .kp = 1000,
        .ki = 1,
        .kd = 0,
        .max_out = 10000,
        .max_iout = 4000,
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
    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);
}
void check_kill_shoot() {
#ifdef HAS_REFEREE
    uint8_t is_shooter_on = referee->game_robot_status.mains_power_shooter_output;
#else
    uint8_t is_shooter_on = 1;
#endif
    if (remote_mode == REMOTE_MODE_KILL || is_shooter_on == 0) {
        steering_motor->Disable();
        flywheel_left->SetTarget(0);
        flywheel_right->SetTarget(0);

        while (remote_mode == REMOTE_MODE_KILL || is_shooter_on == 0) {
#ifdef HAS_REFEREE
            is_shooter_on = referee->game_robot_status.mains_power_shooter_output;
#else
            is_shooter_on = 1;
#endif
            osDelay(1);
        }
    }
    flywheel_left->Enable();
    flywheel_right->Enable();
    steering_motor->Enable();
}