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
#include "referee_task.h"
#include "remote_task.h"
#include "selftest_task.h"
#include "shoot_task.h"

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim7);
    print_use_uart(&huart8, true, 921600);
    init_can();
    init_imu();
    init_buzzer();
    init_selftest();
    init_referee();
    init_remote();
    init_shoot();
    init_gimbal();
    init_chassis();
    // init_ui();
}

void RM_RTOS_Threads_Init(void) {
    extimuTaskHandle = osThreadNew(extimuTask, nullptr, &extimuTaskAttribute);
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
    selftestTaskHandle = osThreadNew(selftestTask, nullptr, &selftestTaskAttribute);
    //    if (ENABLE_UI)
    //        uiTaskHandle = osThreadNew(uiTask, nullptr, &uiTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(3000);
    Buzzer_Sing(DJI);
    char s[50];
    //    char CPU_RunInfo[512];
    while (true) {
        //        print("%.4f %.4f\r\n", yaw_motor->GetTheta(), yaw_motor->GetOmega());
        //        osDelay(2);
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
        print("Mode:%s\r\n", s);
        switch (shoot_fric_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
                strcpy(s, "PREPARE");
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
        switch (shoot_mode) {
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
        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\nSWL: %d SWR: %d "
            "TWL: %d "
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr, dbus->ch4,
            dbus->timestamp);

        print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
        print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
        print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
        print("\r\n");
        print("Shooter Cooling Heat: %hu\r\n",
              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        osDelay(100);
        // print("\r\n");
        // yaw_motor->PrintData();
        // pitch_motor->PrintData();

        //        memset(CPU_RunInfo,0,512);
        //        vTaskList((char *)&CPU_RunInfo); //获取任务运行时间信息
        //        print("---------------------------------------------\r\n");
        //        print("NAME   STATUS   PRIORITY   LESSSTACK   NUM\r\n");
        //        print("%s\r\n", CPU_RunInfo);
        //        print("---------------------------------------------\r\n");
        //        memset(CPU_RunInfo,0,512);
        //        vTaskGetRunTimeStats((char *)&CPU_RunInfo);
        //        print("NAME   COUNT   PERSENT\r\n");
        //        print("%s", CPU_RunInfo);
        //        print("---------------------------------------------\r\n\n");
        //        osDelay(1000);
    }
}
