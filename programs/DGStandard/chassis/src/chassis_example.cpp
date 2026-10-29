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

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dji_dbus.h"

bsp::CAN* can = nullptr;
driver::DjiMotorBase* fl_motor = nullptr;
driver::DjiMotorBase* fr_motor = nullptr;
driver::DjiMotorBase* bl_motor = nullptr;
driver::DjiMotorBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    print_use_uart(&BOARD_UART1);
    can = new bsp::CAN(&hcan2, true);
    fl_motor = new driver::Motor3508(can, 0x202);
    fr_motor = new driver::Motor3508(can, 0x201);
    bl_motor = new driver::Motor3508(can, 0x203);
    br_motor = new driver::Motor3508(can, 0x204);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                                          // 死区
        .A = 3 * PI,                                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    fl_motor->SetMode(driver::DjiMotorBase::OMEGA);
    fl_motor->SetTransmissionRatio(14);

    fr_motor->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    fr_motor->SetMode(driver::DjiMotorBase::OMEGA);
    fr_motor->SetTransmissionRatio(14);

    bl_motor->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    bl_motor->SetMode(driver::DjiMotorBase::OMEGA);
    bl_motor->SetTransmissionRatio(14);

    br_motor->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    br_motor->SetMode(driver::DjiMotorBase::OMEGA);
    br_motor->SetTransmissionRatio(14);

    driver::DjiMotorBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);

    dbus = new remote::DBUS(&huart3);
    HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);  // DBUS initialization needs time

    while (true) {
        const float ratio = 1.0f / 660.0f * 6 * PI;
        // 摇杆中心死区（±10）：回中残留不再产生持续漂移/自转
        const float ch0 = (dbus->ch0 >= -10 && dbus->ch0 <= 10) ? 0.0f : dbus->ch0 * ratio;
        const float ch1 = (dbus->ch1 >= -10 && dbus->ch1 <= 10) ? 0.0f : dbus->ch1 * ratio;
        const float ch4 = (dbus->ch4 >= -10 && dbus->ch4 <= 10) ? 0.0f : dbus->ch4 * ratio;
        // ch4（左波轮）负责转向，避免与云台板的 ch2（云台 yaw）抢同一个摇杆通道
        chassis->SetSpeed(ch0, ch1, ch4);

        // 安全停车：右拨杆 DOWN = 停车；左拨杆留给发射机构（云台板用它切摩擦轮/发射），不再冻结底盘
        if (dbus->swr == remote::DOWN) {
            chassis->SetSpeed(0, 0, 0);
        }

        set_cursor(0, 0);
        clear_screen();

        // Chassis info
        print_enabled("fl_motor", fl_motor->IsOnline());
        print_enabled("fr_motor", fr_motor->IsOnline());
        print_enabled("bl_motor", bl_motor->IsOnline());
        print_enabled("br_motor", br_motor->IsOnline());
        print("\r\n");

        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\nSWL: %d SWR: %d "
            "TWL: %d "
            "@ %d "
            "ms\r\n",
            dbus->ch0,
            dbus->ch1,
            dbus->ch2,
            dbus->ch3,
            dbus->swl,
            dbus->swr,
            dbus->ch4,
            dbus->GetLastUptime()
        );

        chassis->SetPower(false, 30, 20, 60);
        chassis->Update();
        osDelay(10);
    }
}
