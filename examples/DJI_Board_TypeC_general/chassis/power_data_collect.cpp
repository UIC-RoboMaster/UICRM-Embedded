/*###########################################################
 # Copyright (c) 2023-2025. BNU-HKBU UIC RoboMaster         #
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

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "power_meter.h"

bsp::CAN* can = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

power_meter* power_meter_1 = nullptr;

void Dump(void* args);
void callback(uint16_t voltage, int16_t current);
const osThreadAttr_t dump_thread_attr_ = {.name = "DumpThread",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 256 * 4,
                                          .priority = (osPriority_t)osPriorityNormal,
                                          .tz_module = 0,
                                          .reserved = 0};
const bsp::thread_init_t thread_init = {
    .func = Dump,
    .args = nullptr,
    .attr = dump_thread_attr_,
};
bsp::Thread* dump_thread = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&BOARD_UART1, true, 921600);
    can = new bsp::CAN(&hcan2, false);
    fl_motor = new driver::Motor3508(can, 0x201);
    fr_motor = new driver::Motor3508(can, 0x202);
    bl_motor = new driver::Motor3508(can, 0x203);
    br_motor = new driver::Motor3508(can, 0x204);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2000,
        .ki = 0,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 0,
        .deadband = 0,                                          // 死区
        .A = 3 * PI,                                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.05,                   // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fl_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fr_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    bl_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    br_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    br_motor->SetTransmissionRatio(19);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);

    dbus = new remote::DBUS(&BOARD_DBUS);

    power_meter_1 = new power_meter(&BOARD_UART2, 1, callback);

    dump_thread = new bsp::Thread(thread_init);
    dump_thread->Start();

    osDelay(100);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);  // DBUS initialization needs time

    while (true) {
        const float ratio = 1.0f / 660.0f * PI * 12;
        float target = ratio * dbus->ch0;
        fr_motor->SetTarget(target, true);
        osDelay(10);
    }
}

void callback(uint16_t voltage, int16_t current) {
    UNUSED(voltage);
    UNUSED(current);
    dump_thread->Set();
}

void Dump(void* args) {
    UNUSED(args);
    while (true) {
        dump_thread->Wait();
        struct {
            uint8_t header[4] = {0xAA, 0xBB, 0xCC, 0xDD};
            float angular_velocity;
            int16_t cmd_current;
            int16_t read_current;
        } data;
        data.angular_velocity = fr_motor->GetOmega();
        data.cmd_current = fr_motor->GetCurr();
        data.read_current = power_meter_1->current;
        dump(&data, sizeof(data));
    }
}