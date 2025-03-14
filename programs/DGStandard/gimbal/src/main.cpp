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
#include "minipc_task.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "ui_task.h"
/**
 * 在当前版本的程序中，每一个部件都需要作为一个全局的变量被初始化，然后在对应的任务中被使用
 */

bsp::GPIO* gimbal_power = nullptr;
void RM_RTOS_Init(void) {
    // 设置高精度定时器以能够获取微秒级别的精度的运行时间数据
    bsp::SetHighresClockTimer(&htim7);
    // 初始化调试串口，使print()函数能够输出调试信息
    print_use_uart(&huart8, true, 921600);
    // 初始化can总线，can在各个进程中都需要被使用所以在这里独立初始化
    init_can();
    // 初始化IMU
    init_imu();
    // 初始化蜂鸣器，蜂鸣器在需要的时候会在后台播放音乐
    init_buzzer();
    // 初始化裁判系统，裁判系统类能够读取裁判系统的数据
    init_referee();
    // 初始化遥控器与远程模式选择，遥控器类能够读取遥控器的数据
    init_minipc();
    init_remote();
    // 初始化发射机构，发射机构类能够控制发射机构的动作
    init_shoot();
    // 初始化云台，云台类能够控制云台的动作
    init_gimbal();
    // 初始化底盘，底盘类能够控制底盘的动作
    init_chassis();
    // 初始化用户界面，用户界面类能够在图传上显示实时状态
    init_ui();
    gimbal_power = new bsp::GPIO(MOS_CTL2_GPIO_Port, MOS_CTL2_Pin);
    gimbal_power->Low();
}

void RM_RTOS_Threads_Init(void) {
    //    extimuTaskHandle = osThreadNew(extimuTask, nullptr, &extimuTaskAttribute);
    // 分别启动每个任务
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
        if (referee->game_robot_status.mains_power_gimbal_output) {
            gimbal_power->High();
        } else {
            gimbal_power->Low();
        }
        //        print("%.4f %.4f\r\n", yaw_motor->GetTheta(), yaw_motor->GetOmega());
        //        osDelay(2);
        set_cursor(0, 0);
        clear_screen();

        // Mode info
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
        print("Chassis %.3f %.3f %.3f\r\n", chassis->chassis_vx, chassis->chassis_vy,
              chassis->chassis_vt);
        print("Power %.3fV %.3fA %.3fW\r\n", referee->power_heat_data.chassis_volt / 1000.0,
              referee->power_heat_data.chassis_current / 1000.0,
              referee->power_heat_data.chassis_power);
        print("\r\n");

        // Angle info
        print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
              ahrs->INS_angle[2]);
        print("Vision Target: %.3f %.3f [%.2f]", minipc->target_angle.target_pitch,
              minipc->target_angle.target_yaw);
        print("\r\n");

        // Shoot info
        print("Shooter Cooling Heat: %hu\r\n",
              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        print("\r\n");

        // Online info
        print("Comm Stat:  ");
        print("[DBUS %c] ", dbus->IsOnline() ? 'Y' : 'X');
        print("[Referee %c] ", referee->IsOnline() ? 'Y' : 'X');
        print("\r\n");
        print("Motor Stat: ");
        print("[Yaw %c] ", yaw_motor->IsOnline() ? 'Y' : 'X');
        print("[Pitch %c] ", pitch_motor->IsOnline() ? 'Y' : 'X');
        print("\r\n");
        print("Ref Pwr En: ");
        print("[Chassis %c] ", referee->game_robot_status.mains_power_chassis_output ? 'Y' : 'X');
        print("[Gimbal %c] ", referee->game_robot_status.mains_power_gimbal_output ? 'Y' : 'X');
        print("[Shooter %c] ", referee->game_robot_status.mains_power_shooter_output ? 'Y' : 'X');
        print("\r\n");

        osDelay(50);
    }
}
