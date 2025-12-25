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
#include "dbus.h"

// CAN总线
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

// 驱动电机 (3508) - 使用CAN1
driver::MotorCANBase* drive_fl_motor = nullptr;  // 前左驱动
driver::MotorCANBase* drive_fr_motor = nullptr;  // 前右驱动
driver::MotorCANBase* drive_bl_motor = nullptr;  // 后左驱动
driver::MotorCANBase* drive_br_motor = nullptr;  // 后右驱动

// 转向电机 (6020/4310) - 使用CAN2
driver::MotorCANBase* steer_fl_motor = nullptr;  // 前左转向
driver::MotorCANBase* steer_fr_motor = nullptr;  // 前右转向
driver::MotorCANBase* steer_bl_motor = nullptr;  // 后左转向
driver::MotorCANBase* steer_br_motor = nullptr;  // 后右转向

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&huart1);

    // 初始化CAN总线
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, true);

    // 初始化驱动电机 (3508)
    drive_fl_motor = new driver::Motor3508(can1, 0x201);  // CAN1 ID 0x201
    drive_fr_motor = new driver::Motor3508(can1, 0x202);  // CAN1 ID 0x202
    drive_bl_motor = new driver::Motor3508(can1, 0x203);  // CAN1 ID 0x203
    drive_br_motor = new driver::Motor3508(can1, 0x204);  // CAN1 ID 0x204

    // 驱动电机PID参数 - 速度控制
    control::ConstrainedPID::PID_Init_t drive_omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit | control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };

    // 配置驱动电机 - 速度控制模式
    drive_fl_motor->ReInitPID(drive_omega_pid_init, driver::MotorCANBase::OMEGA);
    drive_fl_motor->SetMode(driver::MotorCANBase::OMEGA);
    drive_fl_motor->SetTransmissionRatio(19);

    drive_fr_motor->ReInitPID(drive_omega_pid_init, driver::MotorCANBase::OMEGA);
    drive_fr_motor->SetMode(driver::MotorCANBase::OMEGA);
    drive_fr_motor->SetTransmissionRatio(19);

    drive_bl_motor->ReInitPID(drive_omega_pid_init, driver::MotorCANBase::OMEGA);
    drive_bl_motor->SetMode(driver::MotorCANBase::OMEGA);
    drive_bl_motor->SetTransmissionRatio(19);

    drive_br_motor->ReInitPID(drive_omega_pid_init, driver::MotorCANBase::OMEGA);
    drive_br_motor->SetMode(driver::MotorCANBase::OMEGA);
    drive_br_motor->SetTransmissionRatio(19);

    // 初始化转向电机 (6020) - 具备绝对编码器
    steer_fl_motor = new driver::Motor6020(can2, 0x205);  // CAN2 ID 0x205
    steer_fr_motor = new driver::Motor6020(can2, 0x206);  // CAN2 ID 0x206
    steer_bl_motor = new driver::Motor6020(can2, 0x207);  // CAN2 ID 0x207
    steer_br_motor = new driver::Motor6020(can2, 0x208);  // CAN2 ID 0x208

    // 转向电机PID参数 - 角度控制
    control::ConstrainedPID::PID_Init_t steer_theta_pid_init = {
        .kp = 30,
        .ki = 0,
        .kd = 1.5,
        .max_out = 25000,
        .max_iout = 5000,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit | control::ConstrainedPID::OutputFilter,
    };

    control::ConstrainedPID::PID_Init_t steer_omega_pid_init = {
        .kp = 1500,
        .ki = 1,
        .kd = 0,
        .max_out = 25000,
        .max_iout = 5000,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit | control::ConstrainedPID::OutputFilter,
    };

    // 配置转向电机 - 角度控制模式 (THETA + ABSOLUTE)
    steer_fl_motor->ReInitPID(steer_theta_pid_init, driver::MotorCANBase::THETA);
    steer_fl_motor->ReInitPID(steer_omega_pid_init, driver::MotorCANBase::OMEGA);
    steer_fl_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::ABSOLUTE);
    steer_fl_motor->SetTransmissionRatio(1);  // 6020直接输出，无减速箱

    steer_fr_motor->ReInitPID(steer_theta_pid_init, driver::MotorCANBase::THETA);
    steer_fr_motor->ReInitPID(steer_omega_pid_init, driver::MotorCANBase::OMEGA);
    steer_fr_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::ABSOLUTE);
    steer_fr_motor->SetTransmissionRatio(1);

    steer_bl_motor->ReInitPID(steer_theta_pid_init, driver::MotorCANBase::THETA);
    steer_bl_motor->ReInitPID(steer_omega_pid_init, driver::MotorCANBase::OMEGA);
    steer_bl_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::ABSOLUTE);
    steer_bl_motor->SetTransmissionRatio(1);

    steer_br_motor->ReInitPID(steer_theta_pid_init, driver::MotorCANBase::THETA);
    steer_br_motor->ReInitPID(steer_omega_pid_init, driver::MotorCANBase::OMEGA);
    steer_br_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::ABSOLUTE);
    steer_br_motor->SetTransmissionRatio(1);

    // 配置舵轮底盘 - 8个电机 (45°对角线布局)
    // 轮子编号:
    //        1(前左)          4(前右)
    //              \        /
    //               \  w  /
    //                \  /
    //                 中心
    //                /  \
    //               /    \
    //        2(后左)          3(后右)
    driver::MotorCANBase* motors[control::SteerWheel::motor_num];
    // 驱动电机 (索引 0-3): 对应轮1, 轮4, 轮2, 轮3
    motors[control::SteerWheel::drive_front_left] = drive_fl_motor;   // 轮1 (前左)
    motors[control::SteerWheel::drive_front_right] = drive_fr_motor;  // 轮4 (前右)
    motors[control::SteerWheel::drive_back_left] = drive_bl_motor;    // 轮2 (后左)
    motors[control::SteerWheel::drive_back_right] = drive_br_motor;   // 轮3 (后右)
    // 转向电机 (索引 4-7): 对应轮1, 轮4, 轮2, 轮3
    motors[control::SteerWheel::steer_front_left] = steer_fl_motor;   // 轮1 (前左)
    motors[control::SteerWheel::steer_front_right] = steer_fr_motor;  // 轮4 (前右)
    motors[control::SteerWheel::steer_back_left] = steer_bl_motor;    // 轮2 (后左)
    motors[control::SteerWheel::steer_back_right] = steer_br_motor;   // 轮3 (后右)

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_STEER_WHEEL;
    chassis_data.offset = 0;  // 可根据实际调整
    chassis = new control::Chassis(chassis_data);

    dbus = new remote::DBUS(&huart3);
    HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);  // DBUS initialization needs time

    while (true) {
        // 遥控器输入映射到底盘速度
        // ch0: 左右平移, ch1: 前后平移, ch2: 旋转
        const float ratio = 1.0f / 660.0f * 3 * PI;  // 调整速度系数
        float vx = dbus->ch0 * ratio;
        float vy = dbus->ch1 * ratio;
        float vt = dbus->ch2 * ratio * 0.5f;  // 旋转速度减半

        chassis->SetSpeed(vx, vy, vt);

        // Kill switch - 紧急停止
        if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
            RM_ASSERT_TRUE(false, "Operation killed");
        }

        // 功率限制 (可选)
        chassis->SetPower(false, 80, 60, 60);

        // 更新底盘控制
        chassis->Update();

        osDelay(10);
    }
}
