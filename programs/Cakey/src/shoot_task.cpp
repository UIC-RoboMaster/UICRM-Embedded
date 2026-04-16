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

// PWM摩擦轮电机参数
// Refer to typeA datasheet for channel detail
// https://rm-static.djicdn.com/tem/RoboMaster%E5%BC%80%E5%8F%91%E7%89%88%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf
// https://rm-static.djicdn.com/tem/RoboMaster%20%E5%BC%80%E5%8F%91%E6%9D%BFA%E5%9E%8B%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf
#define LEFT_FLYWHEEL_PWM_CHANNEL 1   // 左摩擦轮PWM通道
#define RIGHT_FLYWHEEL_PWM_CHANNEL 4  // 右摩擦轮PWM通道

// 请参考C615电调使用说明书
// https://rm-static.djicdn.com/tem/17348/RoboMaster%20C615%20%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf
#define TIM_CLOCK_FREQ 1000000  // 定时器时钟频率 (1MHz)
#define MOTOR_OUT_FREQ 500      // 电机输出频率 (500Hz)
#define IDLE_THROTTLE 400       // 摩擦轮停转时PWM脉宽 (SNAIL电调怠速约400us)

static driver::MotorPWMBase* flywheel_left = nullptr;
static driver::MotorPWMBase* flywheel_right = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

bsp::GPIO* shoot_key = nullptr;

bool jam_notify_flags = false;

control::ConstrainedPID::PID_Init_t steering_motor_theta_normal_pid_init = {
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
control::ConstrainedPID::PID_Init_t steering_motor_theta_fast_pid_init = {
    .kp = 25,
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
control::ConstrainedPID::PID_Init_t steering_motor_theta_burst_pid_init = {
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
    // 启动等待
    osDelay(1000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }
    // 等待IMU初始化
    while (ahrs->IsCailbrated()) {
        osDelay(SHOOT_OS_DELAY);
    }

    steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());

    Ease* flywheel_speed_ease = new Ease(0, 0.3);

    while (true) {
        bool shoot_en = true;
        shoot_en &= remote_mode != REMOTE_MODE_KILL;
#ifdef HAS_REFEREE
        shoot_en &= referee->game_robot_status.mains_power_shooter_output;
#endif
        if (!shoot_en) {
            // 死了
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
            shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
            flywheel_speed_ease->SetTarget(900);
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
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        /* 摩擦轮就绪后执行以下部分 */

        if (shoot_load_mode == SHOOT_MODE_STOP) {
            steering_motor->Hold();
            // todo: 这里为什么要是true才能停下？target_angel为什么会超过范围？
        }
        if (shoot_load_mode == SHOOT_MODE_IDLE) {
            uint8_t loaded = shoot_key->Read();
            if (loaded) {
                steering_motor->Hold(true);
            } else {
                // 没有准备就绪，则旋转拔弹电机
                steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, false);
            }
        }

#ifdef HAS_REFEREE
        int heat_limit = referee->game_robot_status.shooter_heat_limit;
        int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
        const int shooter_heat_threashold = 25;
        if (heat_buffer > heat_limit - shooter_heat_threashold) {
            // 临时解决方案
            steering_motor->Hold();
            print("overheat!\n");
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        UNUSED(heat_limit);
        UNUSED(heat_buffer);
#endif

        if (shoot_load_mode == SHOOT_MODE_SINGLE) {
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta() + 2 * PI / 8, true);
            uint8_t loaded = shoot_key->Read();
            // 等到这一粒子弹发射出去再变成IDLE，否则会因为当前有子弹而直接锁定造成无法发射子弹
            if (!loaded) {
                shoot_load_mode = SHOOT_MODE_IDLE;
            }
        }
        if (shoot_load_mode == SHOOT_MODE_BURST) {
            steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, true);
        }

        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::MotorPWMBase(&htim1, LEFT_FLYWHEEL_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                             MOTOR_OUT_FREQ, IDLE_THROTTLE);
    flywheel_right = new driver::MotorPWMBase(&htim1, RIGHT_FLYWHEEL_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                              MOTOR_OUT_FREQ, IDLE_THROTTLE);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
    // 等待电调上电初始化校准完成（SNAIL电调需要在稳定的最低油门信号下完成校准）
    HAL_Delay(3000);

    // C610电调，文档参见：https://www.robomaster.com/zh-CN/products/components/general/M2006
    steering_motor = new driver::Motor2006(can2, 0x207);

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
        .deadband = 0,                                           // 死区
        .A = 3 * PI,                                             // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                                             // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                     // 输出滤波系数
        .derivative_filtering_coefficient = 0,                   // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::ErrorHandle,            // 错误处理

    };
    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);

    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);

    shoot_key = new bsp::GPIO(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
}