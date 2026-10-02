/*
 * 版权所有 (c) 2023-2024 BNU-HKBU UIC RoboMaster。
 *
 * 本程序是自由软件：你可以按照自由软件基金会发布的 GNU 通用公共许可证
 * 第 3 版或（由你选择的）任何后续版本的条款，重新分发和/或修改本程序。
 *
 * 本程序希望能发挥作用，但不提供任何担保，甚至不提供适销性或
 * 特定用途适用性的默示担保。详情请参阅 GNU 通用公共许可证。
 *
 * 你应当已经随本程序收到 GNU 通用公共许可证的副本。
 * 如果没有，请参阅 <https://www.gnu.org/licenses/>。
 */

#include "DjiMotorBase.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dji_dbus.h"
#include "main.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include "vofa.h"

constexpr float kMaxSpeedRadS = 1.0f;  // 满杆目标速度，单位：rad/s。
static bsp::CAN* can1 = nullptr;
static driver::Motor6020* motor1 = nullptr;
static remote::DBUS* dbus = nullptr;
static driver::Vofa vofa;
static float vofa_channels[9] = {};

static void SendPidCurves() {
    const auto omega = motor1->GetPIDState(driver::DjiMotorBase::OMEGA);
    const bool enabled = motor1->IsEnable() && motor1->IsOnline();
    vofa_channels[0] = enabled ? omega.error : 0.0f;          // 速度误差，rad/s。
    vofa_channels[1] = motor1->GetOutputShaftCumulatedTheta() * 180.0f / PI;  // 实际连续角度，度。
    vofa_channels[2] = enabled ? motor1->GetTarget() : 0.0f;  // 目标速度，rad/s。
    vofa_channels[3] = motor1->GetOutputShaftOmega();         // 实际速度，rad/s。
    vofa_channels[4] = enabled ? omega.output : 0.0f;         // PID 控制输出，原始控制量。
    vofa_channels[5] = enabled ? omega.pout : 0.0f;           // P 分量。
    vofa_channels[6] = enabled ? omega.iout : 0.0f;           // I 分量。
    vofa_channels[7] = enabled ? omega.dout : 0.0f;           // D 分量。
    vofa_channels[8] = enabled ? 1.0f : 0.0f;                 // 电机使能状态。
    vofa.Send();
}

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    // USART1 专门发送 JustFloat 波形，不混入文字打印。
    print_use_uart(&huart1, true, 921600);
    vofa.Attach(print_uart);
    for (uint8_t i = 0; i < 9; ++i)
        vofa.BindChannel(i, &vofa_channels[i]);

    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor6020(can1, 0x208);
    motor1->Disable();
    dbus = new remote::DBUS(&BOARD_DBUS);
    dbus->SetThreshold(250);
    motor1->SetTransmissionRatio(1);

    // 纯速度环调参，不启用角度环。
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2000,
        .ki = 20,
        .kd = 8,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,
        .A = 1.5 * PI,
        .B = 1 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
    motor1->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    motor1->SetMode(driver::DjiMotorBase::OMEGA);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    osDelay(500);  // 等待遥控器和电机反馈初始化。
    uint32_t sample_count = 0;
    while (true) {
        if (!dbus->IsOnline() || !motor1->IsOnline() || dbus->swl != remote::MID) {
            motor1->Disable();
        } else {
            if (!motor1->IsEnable()) {
                motor1->SetTarget(0.0f);
                motor1->Enable();
            }
            // 摇杆回中为零速度，满杆为正负 kMaxSpeedRadS。
            const float speed_target =
                -static_cast<float>(dbus->ch2) / remote::DBUS::ROCKER_MAX * kMaxSpeedRadS;
            motor1->SetTarget(speed_target);
        }
        if (++sample_count >= 5) {
            sample_count = 0;
            SendPidCurves();
        }
        osDelay(1);
    }
}
