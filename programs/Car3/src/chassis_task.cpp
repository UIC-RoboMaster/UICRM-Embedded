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
osThreadId_t chassisTaskHandle;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vz = 0;
bool chassis_boost_flag = true;
const float speed_offset = 1320;
const float speed_offset_boost = 2640;
void chassisTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);

    while (remote_mode == REMOTE_MODE_KILL) {
        kill_chassis();
        osDelay(CHASSIS_OS_DELAY);
    }

    while (!imu->DataReady() || !imu->CaliDone()) {
        osDelay(1);
    }

    float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    // float last_speed = 0;
    float sin_yaw, cos_yaw, vx_set = 0, vy_set = 0, vz_set = 0, vx_set_org = 0, vy_set_org = 0;
    float offset_yaw = 0;
    float spin_speed = 350;
    float manual_mode_yaw_pid_args[3] = {300, 0, 0};
    float manual_mode_yaw_pid_max_iout = 0;
    float manual_mode_yaw_pid_max_out = 350;
    control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
        manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
    manual_mode_pid->Reset();
    float manual_mode_pid_output = 0;
    float current_speed_offset = speed_offset;

    remote::keyboard_t keyboard;
    remote::keyboard_t last_keyboard;

    RampSource* vx_ramp = new RampSource(0, -chassis_vx_max / 2, chassis_vx_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    RampSource* vy_ramp = new RampSource(0, -chassis_vy_max / 2, chassis_vy_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    RampSource* vz_ramp = new RampSource(0, -chassis_vz_max / 2, chassis_vz_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));

    BoolEdgeDetector* w_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* s_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* a_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* d_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shift_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* q_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* e_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* x_edge = new BoolEdgeDetector(false);

    BoolEdgeDetector* ch1_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch2_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch3_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch4_edge = new BoolEdgeDetector(false);

    const float ratio = 1.0f / 660.0f * 12 * PI;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            osDelay(CHASSIS_OS_DELAY);

            continue;
        }
        {
            if (!fl_motor->IsEnable() && fl_motor->IsOnline())
                fl_motor->Enable();
            if (!fr_motor->IsEnable() && fr_motor->IsOnline())
                fr_motor->Enable();
            if (!bl_motor->IsEnable() && bl_motor->IsOnline())
                bl_motor->Enable();
            if (!br_motor->IsEnable() && br_motor->IsOnline())
                br_motor->Enable();
            if (fl_motor->IsOnline() && fr_motor->IsOnline() && bl_motor->IsOnline() &&
                br_motor->IsOnline()) {
                chassis->Enable();
            }
        }
        {
            if (sbus->IsOnline()) {
                ch1_edge->input(sbus->ch1 != 0);
                ch2_edge->input(sbus->ch2 != 0);
                ch3_edge->input(sbus->ch3 != 0);
                ch4_edge->input(sbus->ch4 != 0);
            }
            if (refereerc->IsOnline()) {
                last_keyboard = keyboard;
                keyboard = refereerc->remote_control.keyboard;
                w_edge->input(keyboard.bit.W);
                s_edge->input(keyboard.bit.S);
                a_edge->input(keyboard.bit.A);
                d_edge->input(keyboard.bit.D);
                shift_edge->input(keyboard.bit.SHIFT);
                q_edge->input(keyboard.bit.Q);
                e_edge->input(keyboard.bit.E);
                x_edge->input(keyboard.bit.X);
            }
        }

        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);

        // 平移速度控制
        if (ch1_edge->get()) {
            vx_set_org = -sbus->ch1;
        } else if (ch1_edge->negEdge() || x_edge->posEdge()) {
            vx_set_org = 0;
            vx_ramp->SetCurrent(0);
        } else if (d_edge->get()) {
            vx_set_org = vx_ramp->Calc(-current_speed_offset);
        } else if (a_edge->get()) {
            vx_set_org = vx_ramp->Calc(current_speed_offset);
        } else {
            if (vx_set_org > 0) {
                vx_set_org = vx_ramp->Calc(-current_speed_offset);
            } else if (vx_set_org < 0) {
                vx_set_org = vx_ramp->Calc(current_speed_offset);
            }
        }
        // 前进速度控制
        if (ch2_edge->get()) {
            vy_set_org = sbus->ch2;
        } else if (ch2_edge->negEdge() || x_edge->posEdge()) {
            vy_set_org = 0;
            vy_ramp->SetCurrent(0);
        } else if (w_edge->get()) {
            vy_set_org = vy_ramp->Calc(-current_speed_offset);
        } else if (s_edge->get()) {
            vy_set_org = vy_ramp->Calc(current_speed_offset);
        } else {
            if (vy_set_org > 0) {
                vy_set_org = vy_ramp->Calc(-current_speed_offset);
            } else if (vy_set_org < 0) {
                vy_set_org = vy_ramp->Calc(current_speed_offset);
            }
        }
        if (ch4_edge->get() && sbus->ch6 < 0) {
            offset_yaw = sbus->ch4;
        } else if (ch4_edge->negEdge()) {
            offset_yaw = 0;
            vz_ramp->SetCurrent(0);
        } else if (e_edge->get()) {
            offset_yaw = vz_ramp->Calc(current_speed_offset);
        } else if (q_edge->get()) {
            offset_yaw = vz_ramp->Calc(-current_speed_offset);
        } else {
            if (offset_yaw > 0) {
                offset_yaw = vz_ramp->Calc(-current_speed_offset);
            } else if (offset_yaw < 0) {
                offset_yaw = vz_ramp->Calc(current_speed_offset);
            }
        }

        chassis_vx = vx_set_org;
        chassis_vy = vy_set_org;
        chassis_vz = offset_yaw;
        vx_set = cos_yaw * vx_set_org + sin_yaw * vy_set_org;
        vy_set = -sin_yaw * vx_set_org + cos_yaw * vy_set_org;
        switch (remote_mode) {
            case REMOTE_MODE_FOLLOW:
                manual_mode_pid_output = manual_mode_pid->ComputeOutput(
                    yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_));
                chassis->SetSpeed(vx_set * ratio, vy_set * ratio, manual_mode_pid_output * ratio);
                //                chassis->Update(true,
                //                (float)referee->game_robot_status.chassis_power_limit,
                //                                referee->power_heat_data.chassis_power,
                //                                (float)referee->power_heat_data.chassis_power_buffer);
                chassis->SetPower(true, (float)referee->game_robot_status.chassis_power_limit,
                                  referee->power_heat_data.chassis_power,
                                  (float)referee->power_heat_data.chassis_power_buffer);
                chassis->Update();
                break;
            case REMOTE_MODE_SPIN:

                if (offset_yaw != 0) {
                    spin_speed = spin_speed + offset_yaw;
                    offset_yaw = 0;
                    spin_speed = clip<float>(spin_speed, -660, 660);
                }
                vz_set = spin_speed;
                chassis->SetSpeed(vx_set * ratio, vy_set * ratio, vz_set * ratio);
                //                chassis->Update(true,
                //                (float)referee->game_robot_status.chassis_power_limit,
                //                                referee->power_heat_data.chassis_power,
                //                                (float)referee->power_heat_data.chassis_power_buffer);
                chassis->SetPower(true, (float)referee->game_robot_status.chassis_power_limit,
                                  referee->power_heat_data.chassis_power,
                                  (float)referee->power_heat_data.chassis_power_buffer);
                chassis->Update();
                break;
            case REMOTE_MODE_ADVANCED:
                vz_set = offset_yaw;
                chassis->SetSpeed(vx_set * ratio, vy_set * ratio, vz_set * ratio);
                //                chassis->Update(true,
                //                (float)referee->game_robot_status.chassis_power_limit,
                //                                referee->power_heat_data.chassis_power,
                //                                (float)referee->power_heat_data.chassis_power_buffer);
                chassis->SetPower(true, (float)referee->game_robot_status.chassis_power_limit,
                                  referee->power_heat_data.chassis_power,
                                  (float)referee->power_heat_data.chassis_power_buffer);
                chassis->Update();
                break;
            default:
                // Not Support
                kill_chassis();
        }
        chassis_vz = vz_set;
        osDelay(CHASSIS_OS_DELAY);
    }
}

void init_chassis() {
    // 初始化电机
    fl_motor = new driver::Motor3508(can1, 0x202);
    fr_motor = new driver::Motor3508(can1, 0x201);
    bl_motor = new driver::Motor3508(can1, 0x203);
    br_motor = new driver::Motor3508(can1, 0x204);

    // 初始化电机组
    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    // 底盘电机参数设置
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 6000,                              // 变速积分所能达到的最大值为A+B
        .B = 4000,                              // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fl_motor->SetMode(driver::MotorCANBase::OMEGA);
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fr_motor->SetMode(driver::MotorCANBase::OMEGA);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    bl_motor->SetMode(driver::MotorCANBase::OMEGA);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    br_motor->SetMode(driver::MotorCANBase::OMEGA);
    br_motor->SetTransmissionRatio(19);

    // 底盘初始化
    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);
}
void kill_chassis() {
    fl_motor->Disable();
    fr_motor->Disable();
    bl_motor->Disable();
    br_motor->Disable();
}