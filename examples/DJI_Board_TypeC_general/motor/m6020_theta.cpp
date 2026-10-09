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

#include "MotorCanBase.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "vofa.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static driver::Motor6020* motor1 = nullptr;

// ---------------------------------------------------------------------------
// VOFA+ JustFloat 波形输出
// 在 VOFA+ 中按以下顺序配置 5 个通道（JustFloat 协议）：
//   CH0: theta_target  角度环目标 [rad]
//   CH1: theta_actual  角度环实际（编码器角度）[rad]
//   CH2: theta_pid_out 角度环 PID 输出 = 速度环目标 [rad/s]
//   CH3: omega_actual  速度环实际（编码器角速度）[rad/s]
//   CH4: output        最终下发电流指令
// ---------------------------------------------------------------------------
static driver::Vofa vofa;

static float ch_theta_target = 0.f;
static float ch_theta_actual = 0.f;
static float ch_theta_pid_out = 0.f;
static float ch_omega_actual = 0.f;
static float ch_output = 0.f;

static const float* VofaChannels[] = {
    &ch_theta_target, &ch_theta_actual, &ch_theta_pid_out, &ch_omega_actual, &ch_output,
};

static const osThreadAttr_t vofaTaskAttribute = {
    .name = "vofaTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0,
};
static osThreadId_t vofaTaskHandle;

static void vofaTask(void* arg) {
    UNUSED(arg);
    while (true) {
        // 读取电机当前状态
        ch_theta_target = motor1->GetTarget();
        ch_theta_actual = motor1->GetTheta();
        ch_omega_actual = motor1->GetOmega();
        ch_output = static_cast<float>(motor1->GetOutput());
        // 角度环 PID 的输出（即速度环的目标），用于观察串级环路
        ch_theta_pid_out = motor1->GetPIDState(driver::MotorCANBase::THETA).output;

        vofa.Send();
        osDelay(2);  // 500 Hz 发送波形
    }
}

void RM_RTOS_Init() {
    // VOFA+ 走二进制 JustFloat，提高波特率以支持高频波形，且不要再用 print()/clear_screen()
    print_use_uart(&huart6, true, 921600);
    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor6020(can1, 0x20A, 0x2fe);
    motor1->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
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
    motor1->ReInitPID(theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 200,
        .ki = 1,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                                          // 死区
        .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    motor1->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    motor1->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA | driver::MotorCANBase::ABSOLUTE);

    motor1->SetTarget(0);
    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);

    // 绑定 VOFA+ 波形通道（使用 print_uart，与打印互斥）
    vofa.Attach(print_uart);
    vofa.BindChannels(VofaChannels, sizeof(VofaChannels) / sizeof(VofaChannels[0]));
}

void RM_RTOS_Threads_Init(void) {
    vofaTaskHandle = osThreadNew(vofaTask, nullptr, &vofaTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    while (true) {
        // 注意：此处不要调用 set_cursor/clear_screen/PrintData，
        // 否则文本会混入 VOFA+ 的 JustFloat 二进制流，破坏波形。
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            motor1->SetTarget(motor1->GetTarget() + 2 * PI);
            osDelay(20);
        }
        osDelay(20);
    }
}
