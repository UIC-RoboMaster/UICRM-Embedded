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
#include "user_define.h"

//bsp::GPIO* gimbal_power = nullptr;
void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
    print_use_uart(&huart1, true, 921600);
    init_can();
    init_batt();
    init_imu();
    init_buzzer();
    init_referee(); // todo referee线程由父类UARTProtocal构造函数创建，考虑转移到RM_RTOS_Threads_Init
    init_minipc(); //todo minipc线程从这里开始，考虑转移到RM_RTOS_Threads_Init
    init_remote();
    init_shoot();
    init_gimbal();
    init_chassis();
    init_ui();
//    gimbal_power = new bsp::GPIO(MOS_CTL2_GPIO_Port, MOS_CTL2_Pin);
//    gimbal_power->Low();
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
    osDelay(100);
    Buzzer_Sing(DJI);

    while (true) {
        uint8_t buffer[sizeof(control::ConstrainedPID::PID_State_t) * 2 + 2] = {0xAA, 0xBB};

        control::ConstrainedPID::PID_State_t state;
        state = yaw_motor->GetPIDState(driver::MotorCANBase::THETA);
        state.dout = -state.dout;
        memcpy(buffer + 2, &state, sizeof(state));

        state = yaw_motor->GetPIDState(driver::MotorCANBase::OMEGA);
        state.dout = -state.dout;
        memcpy(buffer + 2 + sizeof(state), &state, sizeof(state));

        dump(&state, sizeof(buffer));
        osDelay(4);
    }

    while (true) {
//        if (referee->game_robot_status.mains_power_gimbal_output) {
//            gimbal_power->High();
//        } else {
//            gimbal_power->Low();
//        }
        //        print("%.4f %.4f\r\n", yaw_motor->GetTheta(), yaw_motor->GetOmega());
        //        osDelay(2);
        set_cursor(0, 0);
        clear_screen();

        // Mode info
        print("Mode:%s\r\n", remote_mode_str(remote_mode));
        print("Shoot Fric Mode:%s\r\n", shoot_fric_mode_str(shoot_flywheel_mode));
        print("Shoot Mode:%s\r\n", shoot_load_mode_str(shoot_load_mode));
        print("\r\n");

        // DBUS info
        print(
            "DBUS [CH0: %-4d] [CH1: %-4d] [CH2: %-4d] [CH3: %-4d] [TWL: %d] [SWL: %d] [SWR: %d]"
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->ch4, dbus->swl, dbus->swr,
            dbus->timestamp);
        print("\r\n");

        // Chassis info
        print("Chassis speed %.3f %.3f %.3f\r\n", chassis_vx, chassis_vy, chassis_vt);
        print("Power %.3fV %.3fA %.3fW\r\n", referee->power_heat_data.chassis_volt / 1000.0,
              referee->power_heat_data.chassis_current / 1000.0,
              referee->power_heat_data.chassis_power);
        print("\r\n");

        // Gimbal info
        print("Gimbal target P%.3f Y%.3f\r\n",
              gimbal->getPitchTarget() - gimbal_param->pitch_offset_,
              gimbal->getYawTarget() - gimbal_param->yaw_offset_);
        print("INS Angle: P%.3f Y%.3f R %.3f\r\n", imu->INS_angle[2], imu->INS_angle[0], imu->INS_angle[0]);
        print("Vision Target: P%.3f Y%.3f [%d]\r\n", minipc->target_angle.target_pitch,
              minipc->target_angle.target_yaw, minipc->target_angle.accuracy);
        print("\r\n");

        // Shoot info
        print("Shooter Cooling Heat: %hu\r\n",
              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        print("\r\n");

        // Online info
        print("Comm Stat:  ");
        print_enabled("DBUS", dbus->IsOnline());
        print_enabled("Referee", referee->IsOnline());
        print("\r\n");
        print("Motor Stat: ");
        print_enabled("Yaw", yaw_motor->IsOnline());
        print_enabled("Pitch", pitch_motor->IsOnline());
        print("\r\n");
        print("Ref Pwr En: ");
        print_enabled("Chassis", referee->game_robot_status.mains_power_chassis_output);
        print_enabled("Gimbal", referee->game_robot_status.mains_power_gimbal_output);
        print_enabled("Shooter", referee->game_robot_status.mains_power_shooter_output);
        print("\r\n");

        osDelay(50);
    }
}
