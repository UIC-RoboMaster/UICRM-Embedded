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
const float speed_offset = 660;
const float speed_offset_boost = 1320;
void chassisTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);
    driver::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

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

    RampSource* vx_ramp = new RampSource(0, -chassis_vx_max / 2, chassis_vx_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    RampSource* vy_ramp = new RampSource(0, -chassis_vy_max / 2, chassis_vy_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    RampSource* vz_ramp = new RampSource(0, -chassis_vz_max / 2, chassis_vz_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));

    BoolEdgeDetector* ch1_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch2_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch3_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch4_edge = new BoolEdgeDetector(false);
    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            osDelay(CHASSIS_OS_DELAY);

            continue;
        }
        {
            if (selftest.sbus) {
                ch1_edge->input(sbus->ch1 != 0);
                ch2_edge->input(sbus->ch2 != 0);
                ch3_edge->input(sbus->ch3 != 0);
                ch4_edge->input(sbus->ch4 != 0);
            }
        }

        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);

        if (ch1_edge->get()) {
            vx_set_org = sbus->ch1;
        } else if (ch1_edge->negEdge()) {
            vx_set_org = 0;
            vx_ramp->SetCurrent(0);
        } else {
            if (vx_set_org > 0) {
                vx_set_org = vx_ramp->Calc(-current_speed_offset);
            } else if (vx_set_org < 0) {
                vx_set_org = vx_ramp->Calc(current_speed_offset);
            }
        }
        if (ch2_edge->get()) {
            vy_set_org = sbus->ch2;
        } else if (ch2_edge->negEdge()) {
            vy_set_org = 0;
            vy_ramp->SetCurrent(0);
        } else {
            if (vy_set_org > 0) {
                vy_set_org = vy_ramp->Calc(-current_speed_offset);
            } else if (vy_set_org < 0) {
                vy_set_org = vy_ramp->Calc(current_speed_offset);
            }
        }
        if (sbus->ch6 < 0) {
            if (ch4_edge->get()) {
                offset_yaw = sbus->ch4;
            } else if (ch4_edge->negEdge()) {
                offset_yaw = 0;
                vz_ramp->SetCurrent(0);
            } else {
                if (offset_yaw > 0) {
                    offset_yaw = vz_ramp->Calc(-current_speed_offset);
                } else if (offset_yaw < 0) {
                    offset_yaw = vz_ramp->Calc(current_speed_offset);
                }
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
                chassis->SetSpeed(vx_set, vy_set, manual_mode_pid_output);
                //                chassis->Update(true,
                //                (float)referee->game_robot_status.chassis_power_limit,
                //                                referee->power_heat_data.chassis_power,
                //                                (float)referee->power_heat_data.chassis_power_buffer);
                chassis->SetPower(false, 30, 20, 100);
                chassis->Update();
                break;
            case REMOTE_MODE_SPIN:

                if (offset_yaw != 0) {
                    spin_speed = spin_speed + offset_yaw;
                    offset_yaw = 0;
                    spin_speed = clip<float>(spin_speed, -660, 660);
                }
                vz_set = spin_speed;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
                //                chassis->Update(true,
                //                (float)referee->game_robot_status.chassis_power_limit,
                //                                referee->power_heat_data.chassis_power,
                //                                (float)referee->power_heat_data.chassis_power_buffer);
                chassis->SetPower(false, 30, 20, 100);
                chassis->Update();
                break;
            case REMOTE_MODE_ADVANCED:
                vz_set = offset_yaw;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
                //                chassis->Update(true,
                //                (float)referee->game_robot_status.chassis_power_limit,
                //                                referee->power_heat_data.chassis_power,
                //                                (float)referee->power_heat_data.chassis_power_buffer);
                chassis->SetPower(false, 30, 20, 100);
                chassis->Update();
                break;
            default:
                // Not Support
                kill_chassis();
        }
        chassis_vz = vz_set;
        driver::MotorCANBase::TransmitOutput(motors, 4);
        osDelay(CHASSIS_OS_DELAY);
    }
}

void init_chassis() {
    fl_motor = new driver::Motor3508(can1, 0x202);
    fr_motor = new driver::Motor3508(can1, 0x201);
    bl_motor = new driver::Motor3508(can1, 0x203);
    br_motor = new driver::Motor3508(can1, 0x204);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);
}
void kill_chassis() {
    driver::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
    fl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    br_motor->SetOutput(0);
    driver::MotorCANBase::TransmitOutput(motors, 4);
}