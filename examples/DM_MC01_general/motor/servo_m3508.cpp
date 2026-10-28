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

#include "ServoMotor.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"

bsp::CAN* can1 = NULL;
driver::DjiMotorBase* motor1 = NULL;
driver::ServoMotor* load_servo = NULL;
controller::DBUS* dbus = nullptr;
float load_step_angle = 2 * PI / 8;

void RM_RTOS_Init() {
    bsp::SetHighresClockTimer(&htim5);
    HAL_Delay(200);
    print_use_uart(&huart4);
    dbus = new controller::DBUS(&huart3);
    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor2006(can1, 0x201);

    driver::servo_t servo_data;
    servo_data.motor = motor1;
    servo_data.max_speed = 2.5 * PI;
    servo_data.max_acceleration = 16 * PI;
    servo_data.transmission_ratio = M3508P19_RATIO;
    servo_data.omega_pid_param = new float[3]{6000, 80, 0.3};
    servo_data.max_iout = 6000;
    servo_data.max_out = 32768;
    servo_data.hold_pid_param = new float[3]{150, 10, 0.01};
    servo_data.hold_max_iout = 3000;
    servo_data.hold_max_out = 32768;

    load_servo = new driver::ServoMotor(servo_data);
}

void RM_RTOS_Default_Task(const void* args) {
    using enum controller::ThreePositionSwitch;

    UNUSED(args);
    auto last_state = Mid;
    int state = 0;

    while (true) {
        controller::DbusFrame dbus_state{};
        dbus->GetState(dbus_state);

        if (dbus_state.left_switch == Up) {
            if (last_state == Mid)
                last_state = Up;
        } else if (dbus_state.left_switch == Mid) {
            if (last_state == Up) {
                last_state = Mid;
                load_servo->SetTarget(load_servo->GetTarget() + load_step_angle);
                if (state == 0) {
                    state = 1;
                } else {
                    state = 0;
                }
            }
        }
        load_servo->CalcOutput();

        osDelay(1);
    }
}
