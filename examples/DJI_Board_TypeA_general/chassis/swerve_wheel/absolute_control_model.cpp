/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

/*
 * 差速舵轮单轮组，舵向角由装在舵向轴上的 1:1 AS5047P CAN 磁编提供
 * 与 relative_control_model.cpp 的区别：yaw 有绝对基准
 *
 * 开发板 A 型：两个 3508 在 CAN1，磁编在 CAN2，打印 RTT
 * 上电后电机失能，按板载 KEY 使能，再按失能
 */

#include "AS5047PEncoder.h"
#include "Motor3508.h"
#include "MotorCanBase.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "differential_swerve_wheel.h"
#include "main.h"
#include "tim.h"
#include "utils.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
driver::Motor3508* motor1 = nullptr;
driver::Motor3508* motor2 = nullptr;
driver::AS5047PEncoder* yaw_encoder = nullptr;

namespace {
    constexpr float MOTOR_TRANSMISSION_RATIO = 3.705f;
    constexpr float MAX_YAW_TARGET_SPEED = 6.0f * PI;
    constexpr float MAX_MOTOR_TARGET_SPEED = 30.0f * PI;

    constexpr uint16_t YAW_ENCODER_RX_ID = 0x401;
    constexpr bool YAW_ENCODER_REVERSED = false;
    constexpr uint16_t YAW_ENCODER_ZERO_RAW = 1427;

    // offset 在方向取反之后扣除，所以 reversed 时须同号取负
    constexpr float YAW_ENCODER_OFFSET = (YAW_ENCODER_REVERSED ? -1.0f : 1.0f) *
                                         YAW_ENCODER_ZERO_RAW * 2 * PI /
                                         driver::AS5047PEncoder::COUNTS_PER_REV;

    /*
        Return Yaw Weight 0 ~ 1
        Clamped exponential easing function: [Primary Yaw, Secondary Speed].
        Overall Behaviour:
        Yaw Error   |   Load of Yaw
        HUGE        |   100%
        BIG         |   95%
        MEDIUM      |   90%
        SMALL       |   50%
        TINY        |   0%

        k is the slope function rise from 0 to 1
        k small:    speed weights more
        k big:      yaw weights more
    */
    float CalcYawWeight(float yaw_error) {
        constexpr float k = 2.0f;

        float x = fabsf(yaw_error);
        float y = 1.0f - expf(-k * x);
        if (y > 1.0f)
            y = 1.0f;
        return y;
    }

    void LimitMotorSpeed(float& motor1_omega, float& motor2_omega) {
        const float max_abs = max<float>(fabsf(motor1_omega), fabsf(motor2_omega));
        if (max_abs <= MAX_MOTOR_TARGET_SPEED) {
            return;
        }

        const float scale = MAX_MOTOR_TARGET_SPEED / max_abs;
        motor1_omega *= scale;
        motor2_omega *= scale;
    }

    inline float WrapToPi(float angle) {
        return wrapStrict<float>(angle, -PI, PI);
    }

    inline float ToDeg(float rad) {
        return rad * 180.0f / PI;
    }
}  // namespace

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_rtt();
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
    motor1 = new driver::Motor3508(can1, 0x201);
    motor2 = new driver::Motor3508(can1, 0x202);

    yaw_encoder = new driver::AS5047PEncoder({
        .can = can2,
        .rx_id = YAW_ENCODER_RX_ID,
        .offset = YAW_ENCODER_OFFSET,
        .reversed = YAW_ENCODER_REVERSED,
    });

    control::ConstrainedPID::PID_Init_t omega_pid_init({
        .kp = 100,
        .ki = 0,
        .kd = 1,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit | control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    });
    motor1->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    motor1->SetMode(driver::DjiMotorBase::OMEGA);
    motor1->SetTransmissionRatio(MOTOR_TRANSMISSION_RATIO);

    motor2->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    motor2->SetMode(driver::DjiMotorBase::OMEGA);
    motor2->SetTransmissionRatio(MOTOR_TRANSMISSION_RATIO);

    // DjiMotorState::enable 默认为 true，不显式失能则上电即转
    motor1->Disable();
    motor2->Disable();
    motor1->SetTarget(0);
    motor2->SetTarget(0);

    DWT_Init(168);
    HAL_Delay(500);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    float yaw_target = 0.0f;
    float drive_speed_target = 0.0f;

    while (!yaw_encoder->IsOnline()) {
        print("[init] waiting for yaw encoder, can2 std_id=0x%03X\r\n", YAW_ENCODER_RX_ID);
        osDelay(100);
    }

    const float r1 = motor1->GetTransmissionRatio();
    const float r2 = motor2->GetTransmissionRatio();

    control::DifferentialSwerveWheelKinemetic solver;
    solver.Reset(motor1->GetCumulatedTheta(), motor2->GetCumulatedTheta(), r1, r2);
    solver.AlignYaw(yaw_encoder->GetAngleWrapped(), motor1->GetCumulatedTheta(),
                    motor2->GetCumulatedTheta(), r1, r2);
    print("[init] yaw_initial=%.3f(%.1f°) yaw_bias=%.3f(%.1f°)\r\n", yaw_encoder->GetAngleWrapped(),
          ToDeg(yaw_encoder->GetAngleWrapped()), solver.GetYawBias(), ToDeg(solver.GetYawBias()));

    control::ConstrainedPID yaw_pid({
        .kp = 10,
        .ki = 0,
        .kd = 1,
        .max_out = MAX_YAW_TARGET_SPEED,
        .max_iout = 0,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter,
    });

    bsp::GPIO key(KEY_GPIO_Port, KEY_Pin);
    BoolEdgeDetector encoder_online(true);
    BoolEdgeDetector key_edge(false);
    bool motors_enabled = false;

    uint32_t i = 1;
    short yindex = 0;
    float yaw_targets[] = {0,          PI / 4, PI / 2,     3 * PI / 4, PI,
                           3 * PI / 2, 2 * PI, 3 * PI / 2, PI,         PI / 2};
    short sindex = 0;
    float spd_targets[] = {0, 5, 10, -10, 15, -20};
    while (true) {
        osDelay(5);
        i++;

        const float motor1_theta = motor1->GetCumulatedTheta();
        const float motor2_theta = motor2->GetCumulatedTheta();

        const auto state = solver.Update(motor1_theta, motor2_theta, r1, r2);

        const bool online = yaw_encoder->IsOnline();
        encoder_online.input(online);
        if (encoder_online.posEdge()) {
            solver.AlignYaw(yaw_encoder->GetAngleWrapped(), motor1_theta, motor2_theta, r1, r2);
        }

        const float yaw_from_encoder = yaw_encoder->GetAngleWrapped();
        const float yaw_from_motors = state.yaw_angle;
        const float yaw_measured = online ? yaw_from_encoder : yaw_from_motors;

        key_edge.input(key.Read() == 1);
        if (key_edge.posEdge()) {
            motors_enabled = !motors_enabled;
            if (motors_enabled) {
                // 使能瞬间原地保持，避免朝残留目标猛冲
                yaw_target = yaw_measured;
                drive_speed_target = 0.0f;
                i = 1;
                yindex = 0;
                sindex = 0;
                motor1->Enable();
                motor2->Enable();
            } else {
                motor1->Disable();
                motor2->Disable();
            }
            print("\r\n[key] motors=%s\r\n", motors_enabled ? "on" : "off");
        }

        if (motors_enabled) {
            if (i % 350 == 0) {
                yaw_target = yaw_targets[yindex++ % (sizeof(yaw_targets) / sizeof(float))];
            }
            if (i % 800 == 0) {
                drive_speed_target = spd_targets[sindex++ % (sizeof(spd_targets) / sizeof(float))];
            }
        }

        const float yaw_error = WrapToPi(yaw_target - yaw_measured);
        const float yaw_speed_target =
            yaw_pid.ComputeOutput(yaw_measured + yaw_error, yaw_measured);
        const float drive_speed_loaded = drive_speed_target * (1.0f - CalcYawWeight(yaw_error));

        auto motor_omega_target = solver.InverseSolve(yaw_speed_target, drive_speed_loaded);
        LimitMotorSpeed(motor_omega_target.x1, motor_omega_target.x2);

        if (i % 10 == 0) {
            const float yaw_target_wrapped = WrapToPi(yaw_target);
            const float yaw_mismatch = WrapToPi(yaw_from_encoder - yaw_from_motors);

            print("\r\n");
            print(
                "yaw_target=%.3f(%.1f°) yaw_measured=%.3f(%.1f°) yaw_error=%.3f(%.1f°) "
                "yaw_source=%s motors=%s\r\n",
                yaw_target_wrapped, ToDeg(yaw_target_wrapped), yaw_measured, ToDeg(yaw_measured),
                yaw_error, ToDeg(yaw_error), online ? "encoder" : "motors",
                motors_enabled ? "on" : "off");
            // yaw_mismatch 随转动线性发散即说明 B 矩阵的舵向传动比不对
            print(
                "yaw_from_encoder=%.3f(%.1f°) yaw_from_motors=%.3f(%.1f°) "
                "yaw_mismatch=%.3f(%.1f°)\r\n",
                yaw_from_encoder, ToDeg(yaw_from_encoder), yaw_from_motors, ToDeg(yaw_from_motors),
                yaw_mismatch, ToDeg(yaw_mismatch));
            print(
                "wheel_drive_angle=%.3f(%.1f°) drive_speed_target=%.3f "
                "drive_speed_measured=%.3f\r\n",
                state.drive_angle, ToDeg(state.drive_angle), drive_speed_loaded, state.drive_speed);
            print("encoder_raw_count=%u encoder_omega=%.3f\r\n", yaw_encoder->GetRawCount(),
                  yaw_encoder->GetOmega());
            print(
                "motor1_theta=%.3f(%.1f°) motor2_theta=%.3f(%.1f°) motor1_omega_target=%.3f "
                "motor2_omega_target=%.3f\r\n",
                motor1_theta, ToDeg(motor1_theta), motor2_theta, ToDeg(motor2_theta),
                motor_omega_target.x1, motor_omega_target.x2);
        }

        if (motors_enabled) {
            motor1->SetTarget(motor_omega_target.x1);
            motor2->SetTarget(motor_omega_target.x2);
        } else {
            motor1->SetTarget(0);
            motor2->SetTarget(0);
        }
    }
}
