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
void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);

    // 在这里设置串口号和输出波特率
    print_use_uart(&BOARD_UART1, true, 921600);
    clear_screen();
    print("UART init success!");

    init_can();
    clear_screen();
    print("CAN init success!");

    init_batt();
    clear_screen();
    print("BATTERTY init success!");

    init_imu();
    clear_screen();
    print("IMU init success!");

    init_buzzer();
    clear_screen();
    print("BUZZER init success!");

    init_referee();
    clear_screen();
    print("REFREE init success!");

    init_remote();
    clear_screen();
    print("REMOTE init success!");

    init_shoot();
    clear_screen();
    print("SCREEN init success!");

    init_gimbal();
    clear_screen();
    print("GIMBAL init success!");

    init_chassis();
    clear_screen();
    print("CHASSIS init success!");

    init_ui();
    clear_screen();
    print("UI init success!");

}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
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
    while (true) {
        set_cursor(0, 0);
        clear_screen();


        print("Mode:%s\r\n", remote_mode_str(remote_mode));
        print("Shoot Fric Mode:%s\r\n", shoot_fric_mode_str(shoot_flywheel_mode));
        print("Shoot Mode:%s\r\n", shoot_load_mode_str(shoot_load_mode));
        print("\r\n");

        // Movement info
        print(
            "DBUS [CH0: %-4d] [CH1: %-4d] [CH2: %-4d] [CH3: %-4d] [TWL: %d] [SWL: %d] [SWR: %d]"
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->ch4, dbus->swl, dbus->swr,
            dbus->timestamp);
        print("Chassis %.3f %.3f %.3f\r\n", chassis->chassis_vy, chassis->chassis_vy,
              chassis->chassis_vt);
        print("Power %.3fV %.3fA %.3fW\r\n", referee->power_heat_data.chassis_volt / 1000.0,
              referee->power_heat_data.chassis_current / 1000.0,
              referee->power_heat_data.chassis_power);
        print("\r\n");

        // Shoot info
        print("Shooter Cooling Heat: %hu\r\n",
              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        print("\r\n");

        // Online info
        print("[DBUS %s] ", dbus->IsOnline() ? "\033[32mOnline\033[0m" : "\033[31mOffline\033[0m");
        print("\r\n");
        print("[Referee %s] ", referee->IsOnline() ? "\033[32mOnline\033[0m" : "\033[31mOffline\033[0m");
        print("\r\n");
        print("[Yaw %s] ", yaw_motor->IsOnline() ? "\033[32mOnline\033[0m" : "\033[31mOffline\033[0m");
        print("\r\n");
        print("[Pitch %s] ", pitch_motor->IsOnline() ? "\033[32mOnline\033[0m" : "\033[31mOffline\033[0m");
        print("\r\n");
        print("[Flywheel %s %s]", flywheel_left->IsOnline() ?"\033[32mOnline\033[0m" : "\033[31mOffline\033[0m",flywheel_right->IsOnline() ?"\033[32mOnline\033[0m" : "\033[31mOffline\033[0m");
        print("\r\n");
        osDelay(50);
    }
}
