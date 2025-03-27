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
// #include "shoot_task_CAM.h"
#include "shoot_task.h"
#include "MotorPWMBase.h"
#include "ui_task.h"
#include "tinyML.h"

/**
 * 在当前版本的程序中，每一个部件都需要作为一个全局的变量被初始化，然后在对应的任务中被使用
 */
//
// static ::GPIO* gimbal_power = nullptr;
static driver::MotorPWMBase* Laser = nullptr;

void RM_Car_main_remote_mode(){
    char s[20];
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
        case REMOTE_MODE_ADVANCED:
            strcpy(s, "ADVANCED");
            break;
        case REMOTE_MODE_PREPARE_HAND_MOVEMENT:
            strcpy(s, "STICK_SHIFT");
            break;

        default:
            strcpy(s, "UNKNOWN");
            break;
    }
    print("\n");
    print("Mode:%s\r\n", s);
}

void RM_Car_main_data(bool newline = false){
    print(
    "DBUS [CH0: %-4d] [CH1: %-4d] [CH2: %-4d] [CH3: %-4d] [TWL: %d] [SWL: %d] [SWR: %d]"
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
    print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
          ahrs->INS_angle[2]);
    print("Vision Target: %.3f %.3f\r\n", minipc->target_angle.target_pitch,
          minipc->target_angle.target_yaw);
    if(newline){print("\r\n");}
}

void RM_MODEL_INIT(void) {
    // 初始化can总线，can在各个进程中都需要被使用所以在这里独立初始化
    init_can();
    // 初始化IMU
    IMU_Init();
    // 初始化蜂鸣器，蜂鸣器在需要的时候会在后台播放音乐
    init_buzzer();
    // 初始化裁判系统，裁判系统类能够读取裁判系统的数据
    init_referee();
    // todo:minipc卡住串口跑不了，暂时不用注释掉
    init_minipc();
    // 初始化遥控器与远程模式选择，遥控器类能够读取遥控器的数据
    init_remote();
    // 初始化发射机构，发射机构类能够控制发射机构的动作
    init_shoot();
    // 初始化云台，云台类能够控制云台的动作
    init_gimbal();
    // 初始化底盘，底盘类能够控制底盘的动作
    init_chassis();
    // 初始化用户界面，用户界面类能够在图传上显示实时状态
    init_ui();

    // gimbal_power = new bsp::GPIO(LED_B_GPIO_Port, LED_B_Pin);
    Laser = new driver::Lesar(&htim3, 3,1000000, 50, 0);
    Laser->SetOutput(50);

    print("RM_ALL_INIT_OK\n");
}

void RM_RTOS_Init(void){
    // 设置高精度定时器以能够获取微秒级别的精度的运行时间数据
    bsp::SetHighresClockTimer(&htim8);
    // 初始化调试串口，使print()函数能够输出调试信息
    // print_use_uart(&huart2, true, 115200);
    print("RM_RTOS_UART_INIT\n");
    // 初始化LED灯，LED灯在需要的时候会在后台点亮

    // 初始化调试模式
    RM_MODEL_INIT();
}

void RM_RTOS_Threads_Init(void) {
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

    while (!referee->game_robot_status.mains_power_gimbal_output) {
        print("Waiting model to go online\r\n");
        osDelay(WAIT_CHASSIS_ONLINE_OS_DELAY);
    }
//    while (remote_mode != REMOTE_MODE_KILL) {
//        Buzzer_Sing(ProtectWarning);
//        osDelay(PROTECT_OS_DELAY);
//    }
    protect_wraning_flag = false;

    osDelay(500);
    Buzzer_Sing(DJI);
    while (true) {
        // if (referee->game_robot_status.mains_power_gimbal_output) {
        //     gimbal_power->Low();
        // } else {
        //     gimbal_power->High();
        // }

        RM_Car_main_remote_mode();
        shoot_remote_mode();
        RM_Car_main_data(false);
        debug_gimbal(false);
        debug_chassis(true);
        IMU_print();
        osDelay(50);
    }
}
