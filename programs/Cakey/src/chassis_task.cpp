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

#include "chassis_task.h"

#include <sys/signal.h>

#include "config.h"
osThreadId_t chassisTaskHandle;

const float chassis_max_xy_speed = 2 * PI * 10;
const float chassis_max_t_speed = 2 * PI * 5;

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;
bool chassis_boost_flag = true;

char s[50];

// 这是已经弃用的canbrige，因为云台和底盘现在已经整合在一起了。
// communication::CanBridge* can_bridge = nullptr;
// control::ChassisCanBridgeSender* chassis = nullptr;

// bsp::CAN* can1 = nullptr;
// bsp::CAN* can2 = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;
// 创建没有卵用的超电对象（组长加油！）
driver::SuperCap* super_cap = nullptr;
// 创建用于操作底盘的句柄
control::Chassis* chassis = nullptr;

void init_chassis() {
    HAL_Delay(100);
    // can1 = new bsp::CAN(&hcan1, true);
    // can2 = new bsp::CAN(&hcan2, false);
    fl_motor = new driver::Motor3508(CHASSIS_CAN, FL_MOTOR_ID);
    fr_motor = new driver::Motor3508(CHASSIS_CAN, FR_MOTOR_ID);
    bl_motor = new driver::Motor3508(CHASSIS_CAN, BL_MOTOR_ID);
    br_motor = new driver::Motor3508(CHASSIS_CAN, BR_MOTOR_ID);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = CHASSIS_PID_KP,
        .ki = CHASSIS_PID_KI,
        .kd = CHASSIS_PID_KD,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,  // 死区
        .A = 3 * PI,    // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,    // 启动变速积分的死区
        .output_filtering_coefficient = CHASSIS_PID_FC,         // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    fl_motor->SetMode(driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    fl_motor->SetTransmissionRatio(14);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    fr_motor->SetMode(driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    fr_motor->SetTransmissionRatio(14);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    bl_motor->SetMode(driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    bl_motor->SetTransmissionRatio(14);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    br_motor->SetMode(driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    br_motor->SetTransmissionRatio(14);

    driver::supercap_init_t supercap_init = {
        .can = SUPER_CAP_CAN,
        .tx_id = SUPER_CAP_TX_ID,
        .tx_settings_id = SUPER_CAP_TX_SETTINGS_ID,
        .rx_id = SUPER_CAP_RX_ID,
    };
    super_cap = new driver::SuperCap(supercap_init);
    super_cap->Disable();
    super_cap->TransmitSettings();
    super_cap->Enable();
    super_cap->TransmitSettings();
    super_cap->SetMaxVoltage(24.0f);
    super_cap->SetPowerTotal(100.0f);
    super_cap->SetMaxChargePower(150.0f);
    super_cap->SetMaxDischargePower(250.0f);
    super_cap->SetPerferBuffer(50.0f);

    // can_bridge = new communication::CanBridge(can1, 0x52);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis_data.has_super_capacitor = true;
    chassis_data.super_capacitor = super_cap;
    chassis = new control::Chassis(chassis_data);

    chassis->SetMaxMotorSpeed(2 * PI * 7);

    // chassis->CanBridgeSetTxId(0x51);
    // can_bridge->RegisterRxCallback(0x70, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    // can_bridge->RegisterRxCallback(0x71, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    // can_bridge->RegisterRxCallback(0x72, chassis->CanBridgeUpdateEventPowerLimitWrapper,
    // chassis); can_bridge->RegisterRxCallback(0x73,
    // chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);

    HAL_Delay(300);
    init_buzzer();
}

void chassisTask(void* arg) {
    UNUSED(arg);
    kill_chassis();
    osDelay(1000);

    while (remote_mode == REMOTE_MODE_KILL) {
        kill_chassis();
        osDelay(CHASSIS_OS_DELAY);
    }

    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    chassis->Enable();

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            break;
        }
        remote::keyboard_t keyboard;
        if (dbus->IsOnline()) {
            keyboard = dbus->keyboard;
        } else if (refereerc->IsOnline()) {
            keyboard = refereerc->remote_control.keyboard;
        }

        // 以云台为基准的（整车的）运动速度，范围为[-1, 1]
        float car_vx, car_vy, car_vt;
        //        if (keyboard.bit.X) {
        //            // 刹车
        //            car_vx = 0;
        //            car_vy = 0;
        //            car_vt = 0;
        //        } else
        if (dbus->ch0 || dbus->ch1 || dbus->ch2 || dbus->ch3 || dbus->ch4) {
            // 优先使用遥控器
            car_vx = (float)dbus->ch0 / dbus->ROCKER_MAX;
            car_vy = (float)dbus->ch1 / dbus->ROCKER_MAX;
            car_vt = (float)dbus->ch4 / dbus->ROCKER_MAX;
        } else {
            // 使用键盘
            const float keyboard_speed = keyboard.bit.SHIFT ? 1 : 0.5;
            const float keyboard_spin_speed = 1;
            car_vx = (keyboard.bit.D - keyboard.bit.A) * keyboard_speed;
            car_vy = (keyboard.bit.W - keyboard.bit.S) * keyboard_speed;
            car_vt = (keyboard.bit.E - keyboard.bit.Q) * keyboard_spin_speed;
        }

        // 云台和底盘的角度差
        float chassis_yaw_diff = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        // 底盘以底盘自己为基准的运动速度
        float sin_yaw = arm_sin_f32(chassis_yaw_diff);
        float cos_yaw = arm_cos_f32(chassis_yaw_diff);
        chassis_vx = cos_yaw * car_vx + sin_yaw * car_vy;
        chassis_vy = -sin_yaw * car_vx + cos_yaw * car_vy;
        chassis_vt = 0;

        if (remote_mode == REMOTE_MODE_ADVANCED || remote_mode == REMOTE_MODE_AUTOAIM) {
            // 手动模式下，遥控器直接控制底盘速度
            chassis_vx = car_vx;
            chassis_vy = car_vy;
            chassis_vt = car_vt;
        }

        if (remote_mode == REMOTE_MODE_FOLLOW) {
            // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
            const float angle_threshold = 0.02f;
            float chassis_vt_pid_error = chassis_yaw_diff;
            if (fabs(chassis_vt_pid_error) < angle_threshold) {
                chassis_vt_pid_error = 0;
            }

            static control::ConstrainedPID* chassis_vt_pid =
                new control::ConstrainedPID(4 / (2 * PI), 0, 0, 0.5, 1);
            float vt = chassis_vt_pid->ComputeOutput(chassis_vt_pid_error);
            if (chassis_vt_pid_error != 0)
                chassis_vt = vt;
        }

        if (remote_mode == REMOTE_MODE_SPIN) {
            // 小陀螺模式，拨盘用来控制底盘加速度
            static float spin_speed = 1;
            spin_speed = spin_speed + car_vt * 0.01;
            spin_speed = clip<float>(spin_speed, -1, 1);
            chassis_vt = spin_speed;
        }

        // 进行缩放
        chassis_vx *= chassis_max_xy_speed;
        chassis_vy *= chassis_max_xy_speed;
        chassis_vt *= chassis_max_t_speed;

        static const float move_ease_ratio = 1.8;
        static const float turn_ease_ratio = 0.9;
        static Ease chassis_ease_vx(0, move_ease_ratio);
        static Ease chassis_ease_vy(0, move_ease_ratio);
        static Ease chassis_ease_vt(0, turn_ease_ratio);
        chassis_vx = chassis_ease_vx.Calc(chassis_vx);
        chassis_vy = chassis_ease_vy.Calc(chassis_vy);
        chassis_vt = chassis_ease_vt.Calc(chassis_vt);

        chassis->SetSpeed(chassis_vx, chassis_vy, chassis_vt);
        osDelay(CHASSIS_OS_DELAY);
        chassis->SetPower(false, referee->game_robot_status.chassis_power_limit,
                          referee->power_heat_data.chassis_power,
                          referee->power_heat_data.chassis_power_buffer, false);
        chassis->Update();
        osDelay(CHASSIS_OS_DELAY);
    }
}

void kill_chassis() {
    chassis->Disable();
}