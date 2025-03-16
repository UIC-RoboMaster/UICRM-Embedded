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
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "public_port.h"
// #include "referee_task.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "ui_task.h"
#include "user_define.h"
#include "minipc_task.h"
void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
    print_use_uart(&huart1, true, 921600);
    init_can();
    init_batt();
    init_imu();
    init_buzzer();
    init_referee();
    init_minipc(); //todo minipc线程从这里开始，考虑转移到RM_RTOS_Threads_Init
    init_remote();
    init_shoot();
    init_gimbal();
    init_chassis();
    init_ui();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    // refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
    // refereercTaskHandle = osThreadNew(refereercTask, nullptr, &refereercTaskAttribute);
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
    if (ENABLE_UI)
        uiTaskHandle = osThreadNew(uiTask, nullptr, &uiTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(3000);
    Buzzer_Sing(DJI);
//    Buzzer_Sing(War_Cant_of_Mars);
    // while (true) {
    //     clear_screen();
    //     set_cursor(0, 0);
    //     flywheel_left->PrintData();
    //     flywheel_right->PrintData();
    //     osDelay(10);
    // }
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        char remoteModeString[20] = {};
        char fricModeString[20] = {};
        char shootModeString[20] = {};
        switch (remote_mode) {
            case REMOTE_MODE_PREPARE:
                strcpy(remoteModeString, "PREPARE");
                break;
            case REMOTE_MODE_STOP:
                strcpy(remoteModeString, "STOP");
                break;
            case REMOTE_MODE_KILL:
                strcpy(remoteModeString, "KILL");
                break;
            case REMOTE_MODE_FOLLOW:
                strcpy(remoteModeString, "FOLLOW");
                break;
            case REMOTE_MODE_SPIN:
                strcpy(remoteModeString, "SPIN");
                break;
            case REMOTE_MODE_AUTOPILOT:
                strcpy(remoteModeString, "AUTOPILOT");
                break;
            default:
                strcpy(remoteModeString, "UNKNOWN");
                break;
        }
        switch (shoot_flywheel_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
                strcpy(fricModeString, "PREPARE");
                break;
            case SHOOT_FRIC_MODE_STOP:
                strcpy(fricModeString, "STOP");
                break;
            case SHOOT_FRIC_MODE_PREPARED:
                strcpy(fricModeString, "PREPARED");
                break;
            case SHOOT_FRIC_MODE_DISABLE:
                strcpy(fricModeString, "DISABLE");
                break;
            case SHOOT_FRIC_MODE_SPEEDUP:
                strcpy(fricModeString, "SPEEDUP");
                break;
            case SHOOT_FRIC_MODE_SPEEDOWN:
                strcpy(fricModeString, "SPEEDOWN");
                break;
            default:
                strcpy(fricModeString, "UNKNOWN");
                break;
        }
        switch (shoot_load_mode) {
            case SHOOT_MODE_PREPARING:
                strcpy(shootModeString, "PREPARE");
                break;
            case SHOOT_MODE_STOP:
                strcpy(shootModeString, "STOP");
                break;
            case SHOOT_MODE_PREPARED:
                strcpy(shootModeString, "PREPARED");
                break;
            case SHOOT_MODE_DISABLE:
                strcpy(shootModeString, "DISABLE");
                break;
            case SHOOT_MODE_SINGLE:
                strcpy(shootModeString, "SINGLE");
                break;
            case SHOOT_MODE_BURST:
                strcpy(shootModeString, "BURST");
                break;
            default:
                strcpy(shootModeString, "UNKNOWN");
                break;
        }

        print("clear\n");

        print("Mode:%s\r\n", remoteModeString);
        print("Shoot Flywheel Mode:%s\r\n", fricModeString);
        print("Shoot Mode:%s\r\n", shootModeString);
        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", imu->Temp);
        print("Heater: %.2f\r\n", imu->TempPWM);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
              imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);
        print("Is Calibrated: %s\r\n",
              imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m");
        print("Yaw Motor: %.2f, %.2f\r\n", yaw_motor->GetTheta(), yaw_motor->GetOmega());
        print("Pitch Motor: %.2f, %.2f\r\n", pitch_motor->GetTheta(), pitch_motor->GetOmega());
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
//        print("\r\n");
//        print("Vision Target: %.3f %.3f\r\n", minipc->target_angle.target_pitch, minipc->target_angle.target_yaw);
//        print("Vision accuracy: %.3f", minipc->target_angle.accuracy);
        osDelay(75);
    }
}
