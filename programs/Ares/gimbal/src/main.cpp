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

#include "main.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "public_port.h"
// #include "referee_task.h"
#include "remote_task.h"
#include "shoot_task.h"
// #include "ui_task.h"
#include "user_define.h"
void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);

    //print_use_uart(&huart1, true, 921600);
    print_use_rtt();
    init_can();
    // init_batt();
    init_imu();
    init_buzzer();
    // init_referee();
    init_remote();
    init_shoot();
    init_gimbal();
    init_chassis();
    // init_ui();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    // refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);        // 裁判系统
    // refereercTaskHandle = osThreadNew(refereercTask, nullptr, &refereercTaskAttribute);  // 裁判系统图传链路
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
    // if (ENABLE_UI)
    //     uiTaskHandle = osThreadNew(uiTask, nullptr, &uiTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(3000);
    Buzzer_Sing(DJI);
    // while (true) {
    //     clear_screen();
    //     set_cursor(0, 0);
    //     flywheel_left->PrintData();
    //     flywheel_right->PrintData();
    //     osDelay(10);
    // }
    char s[50];
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        switch (remote_mode) {
            case REMOTE_MODE_PREPARE:
                strcpy(s, "PREPARE");
                break;
            case REMOTE_MODE_STOP:
                strcpy(s, "STOP");
                break;
            case REMOTE_MODE_KILL:
                strcpy(s, "KILL");
                break;
            case REMOTE_MODE_FOLLOW:
                strcpy(s, "MANUAL");
                break;
            case REMOTE_MODE_SPIN:
                strcpy(s, "SPIN");
                break;
            default:
                strcpy(s, "UNKNOWN");
                break;
        }
        print("\n");
        print("Mode:%s\r\n", s);
        // switch (shoot_fric_mode) {
        //     case SHOOT_FRIC_MODE_PREPARING:
        //         strcpy(s, "PREPARE");
        //         break;
        //     case SHOOT_FRIC_MODE_STOP:
        //         strcpy(s, "STOP");
        //         break;
        //     case SHOOT_FRIC_MODE_PREPARED:
        //         strcpy(s, "PREPARED");
        //         break;
        //     case SHOOT_FRIC_MODE_DISABLE:
        //         strcpy(s, "DISABLE");
        //         break;
        // }
        // print("Shoot Fric Mode:%s\r\n", s);
        switch (shoot_load_mode) {
            case SHOOT_MODE_PREPARING:
                strcpy(s, "PREPARE");
                break;
            case SHOOT_MODE_STOP:
                strcpy(s, "STOP");
                break;
            case SHOOT_MODE_PREPARED:
                strcpy(s, "PREPARED");
                break;
            case SHOOT_MODE_DISABLE:
                strcpy(s, "DISABLE");
                break;
            case SHOOT_MODE_SINGLE:
                strcpy(s, "SINGLE");
                break;
            case SHOOT_MODE_BURST:
                strcpy(s, "BURST");
                break;
        }


        print("Shoot Mode:%s\r\n", s);

        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", imu->Temp);
        print("Heater: %.2f\r\n", imu->TempPWM);
        print("Angles: yaw %.2f, pitch %.2f, roll %.2f\r\n", imu->INS_angle[0], imu->INS_angle[1] , imu->INS_angle[2]);
        print("Euler Angles: yaw %.2f, pitch %.2f, roll %.2f\r\n", imu->INS_angle[0] / PI * 180,
              imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);
        print("Is Calibrated: %s\r\n",
              imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m");

        // Gimbal info
        print("Gimbal target Pitch %.3f Yaw %.3f\r\n",
              wrap<float>(gimbal->getPitchTarget() - gimbal_param->pitch_offset_, -PI, PI),
              wrap<float>(gimbal->getYawTarget() - gimbal_param->yaw_offset_, -PI, PI));
        print("INS Angle: yaw %.3f pitch %.3f roll %.3f\r\n", pitch_curr, yaw_curr, imu->INS_angle[2]);
        print("\r\n");

        // // 添加在 print("Yaw Motor: ...") 附近
        // print("=== Chassis Debug ===\r\n");
        // print("A (yaw_motor - offset): %.4f\r\n", yaw_motor->GetTheta() - gimbal_param->yaw_offset_);
        // print("B (IMU yaw): %.4f\r\n", imu->INS_angle[0]);
        // print("C (gimbal target): %.4f\r\n", gimbal->getYawTarget());
        // print("chassis_target_diff: %.4f\r\n", gimbal->getYawTarget() - imu->INS_angle[0] + (yaw_motor->GetTheta() - gimbal_param->yaw_offset_));
        // print("yaw_offset_: %.4f\r\n", gimbal_param->yaw_offset_);

        print_enabled("yaw", yaw_motor->IsOnline());
        print("Yaw Motor: %.2f, %.2f\r\n", yaw_motor->GetTheta(), yaw_motor->GetOmega());
        print("yaw current i: %.2f\r\n", yaw_motor->GetCurr());

        print_enabled("pitch", pitch_motor->IsOnline());
        print("Pitch Motor: %.2f, %.2f\r\n", pitch_motor->GetTheta(), pitch_motor->GetOmega());
        print("pitch current i: %.2f\r\n", pitch_motor->GetCurr());

        // // === Pitch Debug ===
        // print("=== Pitch Debug ===\r\n");
        // print("pitch_curr  (IMU): %.4f rad / %.2f deg\r\n", pitch_curr, pitch_curr / PI * 180);
        // print("pitch_target     : %.4f rad / %.2f deg\r\n", pitch_target, pitch_target / PI * 180);
        // print("pitch_diff       : %.4f rad / %.2f deg\r\n", pitch_diff, pitch_diff / PI * 180);
        // print("pitch motor theta: %.4f rad / %.2f deg\r\n", pitch_motor->GetTheta(), pitch_motor->GetTheta() / PI * 180);
        // print("pitch motor omega: %.4f\r\n", pitch_motor->GetOmega());
        // print("pitch motor curr : %.2f\r\n", pitch_motor->GetCurr());
        // print("pitch_offset     : %.4f\r\n", gimbal_param->pitch_offset_);
        // print("pitch_max        : %.4f\r\n", gimbal_param->pitch_max_);
        // print("\r\n");

        // 发射供弹
        print("flywheel_left Motor: %.2f, %.2f\r\n", flywheel_left->GetTheta(), flywheel_left->GetOmega());
        print_enabled("flywheel_left", flywheel_left->IsOnline());

        print("flywheel_right Motor: %.2f, %.2f\r\n", flywheel_right->GetTheta(), flywheel_right->GetOmega());
        print_enabled("flywheel_right", flywheel_right->IsOnline());

        print("steering Motor: %.2f, %.2f\r\n", steering_motor->GetTheta(), steering_motor->GetOmega());
        print_enabled("steering_motor", steering_motor->IsOnline());

        print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
        print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
        print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
        print("\r\n");
        print("Shooter Cooling Heat: %hu\r\n",
              referee->power_heat_data.shooter_id1_42mm_cooling_heat);
        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        print("\r\n");
        print("Current HP %d/%d\n", referee->game_robot_status.remain_HP,
              referee->game_robot_status.max_HP);
        print("Remain bullet %d\n", referee->bullet_remaining.bullet_remaining_num_42mm);
        print("\n");


        osDelay(100);
    }
}
