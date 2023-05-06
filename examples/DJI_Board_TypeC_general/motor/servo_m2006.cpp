/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"

bsp::CAN* can2 = NULL;
control::MotorCANBase* motor1 = NULL;
control::ServoMotor* load_servo = NULL;
remote::DBUS* dbus = nullptr;
float load_step_angle = 2 * PI / 8;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&htim5);
    HAL_Delay(200);
    print_use_uart(&huart6);
    dbus = new remote::DBUS(&huart3);
    can2 = new bsp::CAN(&hcan2, 0x207, false);
    motor1 = new control::Motor2006(can2, 0x207);
    control::servo_t servo_data;

    servo_data.motor = motor1;
    servo_data.max_speed = 2.5 * PI;
    servo_data.max_acceleration = 16 * PI;
    servo_data.transmission_ratio = M2006P36_RATIO;
    servo_data.omega_pid_param = new float[3]{6000, 80, 0.3};
    servo_data.max_iout = 4000;
    servo_data.max_out = 10000;
    servo_data.hold_pid_param = new float[3]{150, 2, 0.01};
    servo_data.hold_max_iout = 2000;
    servo_data.hold_max_out = 10000;

    load_servo = new control::ServoMotor(servo_data);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    control::MotorCANBase* motors[] = {motor1};
    int last_state = remote::MID;
    int state = 0;

    while (true) {
        if (dbus->swl == remote::UP) {
            if (last_state == remote::MID)
                last_state = remote::UP;
        } else if (dbus->swl == remote::MID) {
            if (last_state == remote::UP) {
                last_state = remote::MID;
                load_servo->SetTarget(load_servo->GetTarget() + load_step_angle);
                if (state == 0) {
                    state = 1;
                } else {
                    state = 0;
                }
            }
        }
        load_servo->CalcOutput();

        control::MotorCANBase::TransmitOutput(motors, 1);
        osDelay(1);
    }
}
