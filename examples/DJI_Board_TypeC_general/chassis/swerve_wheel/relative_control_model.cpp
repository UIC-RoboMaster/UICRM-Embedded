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
#include "Motor3508.h"
#include "MotorCanBase.h"
#include "bsp_dwt.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "differential_swerve_wheel.h"
#include "main.h"
#include "tim.h"
#include "utils.h"
#include "bsp_gpio.h"

bsp::CAN* can = nullptr;
driver::Motor3508* motor1 = nullptr;
driver::Motor3508* motor2 = nullptr;

namespace {
    constexpr float MOTOR_TRANSMISSION_RATIO = 3.705f;
    constexpr float MAX_YAW_TARGET_SPEED = 6.0f * PI;
    constexpr float MAX_MOTOR_TARGET_SPEED = 30.0f * PI;

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
    float CalcYawWeight(float yaw_error)
    {
        constexpr float k = 8.0f;

        float x = fabsf(yaw_error);
        float y = 1.0f - expf(-k * x);
        if (y > 1.0f) y = 1.0f;
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

    //TODO: SERIOUSLY, THIS IS THE CORRECT VERSION OF Wrap()
    inline float WrapToPi(float angle)
    {
        while (angle > PI)
        {
            angle -= 2.0f * PI;
        }

        while (angle <= -PI)
        {
            angle += 2.0f * PI;
        }

        return angle;
    }
}  // namespace

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&huart6, true, 921600);
    bsp::SetHighresClockTimer(&htim5);
    can = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor3508(can, 0x201);
    motor2 = new driver::Motor3508(can, 0x202);

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
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    });
    motor1->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    motor1->SetMode(driver::DjiMotorBase::OMEGA);
    motor1->SetTransmissionRatio(MOTOR_TRANSMISSION_RATIO);

    motor2->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    motor2->SetMode(driver::DjiMotorBase::OMEGA);
    motor2->SetTransmissionRatio(MOTOR_TRANSMISSION_RATIO);

    DWT_Init(168);
    HAL_Delay(500);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    float yaw_target = 0.0f;
    float driving_speed_target = 10.0f;

    control::DifferentialSwerveWheelKinemetic solver;
    solver.Reset(motor1->GetTheta(), motor2->GetTheta());

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

    uint32_t i = 1;
    short yindex = 0;
    float yaw_targets[] = {0, PI/2, PI, 3*PI/2, 2*PI, 3*PI/2, PI, PI/2};
    // const float targets[] = {0, PI/2};
    short sindex = 0;
    float spd_targets[] = {0, 5, 10, -10, 15, -20};
    short keyState = 2;
    while (true) {
        osDelay(5);

        if (i++ % 500 == 0) {
            // yaw_target += PI * 0.5f;
            yaw_target = yaw_targets[yindex++ % (sizeof(yaw_targets)/sizeof(float))];
        }
        if (i % 800 == 0) {
            driving_speed_target = spd_targets[sindex++ % (sizeof(spd_targets)/sizeof(float))];
        }

        const float m1 = motor1->GetCumulatedTheta();
        const float m2 = motor2->GetCumulatedTheta();

        const auto state = solver.Update(m1, m2, motor1->GetTransmissionRatio(), motor2->GetTransmissionRatio());
        const float yaw_error = WrapToPi(yaw_target - state.yaw_angle_raw);
        const float wrapped_yaw_target = state.yaw_angle_raw + yaw_error;
        const float yaw_speed_target = yaw_pid.ComputeOutput(wrapped_yaw_target, state.yaw_angle_raw);
        const float load_balanced_drive_speed = driving_speed_target * (1.0f - CalcYawWeight(yaw_error));

        auto target = solver.InverseSolve(yaw_speed_target, load_balanced_drive_speed);
        LimitMotorSpeed(target.x1, target.x2);

        print("\r\n");
        print("[YAW]target:%.3f | current:%.3f | err:%.3f\r\n", yaw_target, state.yaw_angle_raw,  yaw_error);
        print("[SPD]target:%.3f | current:%.3f\r\n", load_balanced_drive_speed, state.drive_speed);
        print("m1:%.3f m2:%.3f\r\n", m1, m2);

        motor1->SetTarget(target.x1);
        motor2->SetTarget(target.x2);

        if (keyState == 0) {
            motor1->Enable();
            motor2->Enable();
        }
        else if (keyState == 3) {
            motor1->Disable();
            motor2->Disable();
        }

        if (key.Read() == 1) {
            if (keyState == 0) keyState = 1;
            else if (keyState == 2) keyState = 3;
            continue;
        }
        if (keyState == 1) {
            keyState = 2;
        }
        if (keyState == 3) {
            keyState = 0;
        }
    }
}

