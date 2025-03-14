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

// 开关功率控制！ true为开false为关
#define POWER_LIMIT_STATUS false

#include "chassis_task.h"
osThreadId_t chassisTaskHandle;

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vz = 0;
bool chassis_boost_flag = true;
const float speed_offset = 660;
const float speed_offset_boost = 1320;

communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;

void chassisTask(void* arg) {
    UNUSED(arg);
    kill_chassis();
    osDelay(1000);

    while (remote_mode == REMOTE_MODE_KILL) {
        kill_chassis();
        osDelay(CHASSIS_OS_DELAY);
    }

    while (!imu->CaliDone()) {
        osDelay(1);
    }

    float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    // float last_speed = 0;
    float sin_yaw, cos_yaw, vx_set = 0, vy_set = 0, vz_set = 0, vx_set_org = 0, vy_set_org = 0;
    float offset_yaw = 0;
    float spin_speed = 350;
    float manual_mode_yaw_pid_args[3] = {400, 0.5, 20};
    float manual_mode_yaw_pid_max_iout = 100;
    float manual_mode_yaw_pid_max_out = 500;
    control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
        manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
    manual_mode_pid->Reset();
    float yaw_pid_error = 0;
    float manual_mode_pid_output = 0;
    float current_speed_offset = speed_offset;
    remote::keyboard_t keyboard;
    remote::keyboard_t last_keyboard;
    RampSource* vx_ramp = new RampSource(0, -chassis_vx_max / 2, chassis_vx_max / 2, 0.01);
    RampSource* vy_ramp = new RampSource(0, -chassis_vy_max / 2, chassis_vy_max / 2, 0.01);
    RampSource* vz_ramp = new RampSource(0, -chassis_vz_max / 2, chassis_vz_max / 2, 0.01);
    BoolEdgeDetector* w_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* s_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* a_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* d_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ctrl_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* q_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* e_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* x_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch0_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch1_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch2_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch3_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch4_edge = new BoolEdgeDetector(false);

    chassis->Enable();

    const float ratio = 1.0f / 660.0f * 10 * PI;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            while (remote_mode == REMOTE_MODE_KILL) {
                osDelay(CHASSIS_OS_DELAY + 2);
            }
            chassis->Enable();
            continue;
        }
        // 更新状态机
        {
            last_keyboard = keyboard;
            if (dbus->IsOnline()) {
                keyboard = dbus->keyboard;
                ch0_edge->input(dbus->ch0 != 0);
                ch1_edge->input(dbus->ch1 != 0);
                ch2_edge->input(dbus->ch2 != 0);
                ch3_edge->input(dbus->ch3 != 0);
                ch4_edge->input(dbus->ch4 != 0);
            } else if (refereerc->IsOnline()) {
                keyboard = refereerc->remote_control.keyboard;
                ch0_edge->input(false);
                ch1_edge->input(false);
                ch2_edge->input(false);
                ch3_edge->input(false);
                ch4_edge->input(false);
            }
        }
        {
            w_edge->input(keyboard.bit.W);
            s_edge->input(keyboard.bit.S);
            a_edge->input(keyboard.bit.A);
            d_edge->input(keyboard.bit.D);
            ctrl_edge->input(keyboard.bit.CTRL);
            q_edge->input(keyboard.bit.Q);
            e_edge->input(keyboard.bit.E);
            x_edge->input(keyboard.bit.X);
        }

        // 更新云台角度
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        // 计算角度的sin/cos
        sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);
        // 检测到ctrl，切换速度模式
        if (ctrl_edge->posEdge()) {
            if (chassis_boost_flag) {
                chassis_boost_flag = false;
                vx_ramp->SetMax(chassis_vx_max / 2);
                vy_ramp->SetMax(chassis_vy_max / 2);
                vz_ramp->SetMax(chassis_vz_max / 2);
                vx_ramp->SetMin(-chassis_vx_max / 2);
                vy_ramp->SetMin(-chassis_vy_max / 2);
                vz_ramp->SetMin(-chassis_vz_max / 2);
                current_speed_offset = speed_offset;
            } else {
                chassis_boost_flag = true;
                vx_ramp->SetMax(chassis_vx_max);
                vy_ramp->SetMax(chassis_vy_max);
                vz_ramp->SetMax(chassis_vz_max);
                vx_ramp->SetMin(-chassis_vx_max);
                vy_ramp->SetMin(-chassis_vy_max);
                vz_ramp->SetMin(-chassis_vz_max);
                current_speed_offset = speed_offset_boost;
            }
        }
        // 平移速度控制
        if (ch0_edge->get()) {
            vx_set_org = dbus->ch0;
        } else if (ch0_edge->negEdge() || x_edge->posEdge()) {
            vx_set_org = 0;
            vx_ramp->SetCurrent(0);
        } else if (d_edge->get()) {
            vx_set_org = vx_ramp->Calc(current_speed_offset);
        } else if (a_edge->get()) {
            vx_set_org = vx_ramp->Calc(-current_speed_offset);
        } else {
            if (vx_set_org > 0) {
                vx_set_org = vx_ramp->Calc(-current_speed_offset);
            } else if (vx_set_org < 0) {
                vx_set_org = vx_ramp->Calc(current_speed_offset);
            }
        }
        // 前进速度控制
        if (ch1_edge->get()) {
            vy_set_org = dbus->ch1;
        } else if (ch1_edge->negEdge() || x_edge->posEdge()) {
            vy_set_org = 0;
            vy_ramp->SetCurrent(0);
        } else if (w_edge->get()) {
            vy_set_org = vy_ramp->Calc(current_speed_offset);
        } else if (s_edge->get()) {
            vy_set_org = vy_ramp->Calc(-current_speed_offset);
        } else {
            if (vy_set_org > 0) {
                vy_set_org = vy_ramp->Calc(-current_speed_offset);
            } else if (vy_set_org < 0) {
                vy_set_org = vy_ramp->Calc(current_speed_offset);
            }
        }
        // 旋转速度控制（如果需要）
        if (ch4_edge->get()) {
            offset_yaw = dbus->ch4;
        } else if (ch4_edge->negEdge() || x_edge->posEdge()) {
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
        if (abs(vx_set_org) < 0.05f) {
            vx_set_org = 0;
        }
        if (abs(vy_set_org) < 0.05f) {
            vy_set_org = 0;
        }
        if (abs(offset_yaw) < 0.05f) {
            offset_yaw = 0;
        }
        chassis_vx = vx_set_org * ratio;
        chassis_vy = vy_set_org * ratio;
        chassis_vz = offset_yaw * ratio;
        vx_set = cos_yaw * chassis_vx + sin_yaw * chassis_vy;
        vy_set = -sin_yaw * chassis_vx + cos_yaw * chassis_vy;
        switch (remote_mode) {
            case REMOTE_MODE_FOLLOW:
                yaw_pid_error = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

                manual_mode_pid_output = manual_mode_pid->ComputeOutput(yaw_pid_error);
                vz_set = manual_mode_pid_output * ratio;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
                osDelay(1);
                chassis->SetPower(POWER_LIMIT_STATUS, referee->game_robot_status.chassis_power_limit,
                                  referee->power_heat_data.chassis_power,
                                  referee->power_heat_data.chassis_power_buffer);
                osDelay(1);
                vx_set = chassis_vx;
                vy_set = chassis_vy;
                vz_set = chassis_vz;
                break;
            case REMOTE_MODE_SPIN:

                if (offset_yaw != 0) {
                    spin_speed = spin_speed + offset_yaw;
                    offset_yaw = 0;
                    spin_speed = clip<float>(spin_speed, -660, 660);
                }
                vz_set = spin_speed * ratio;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
                osDelay(1);
                chassis->SetPower(POWER_LIMIT_STATUS, referee->game_robot_status.chassis_power_limit,
                                  referee->power_heat_data.chassis_power,
                                  referee->power_heat_data.chassis_power_buffer);
                osDelay(1);
                vx_set = chassis_vx;
                vy_set = chassis_vy;
                vz_set = chassis_vz;

                break;
            case REMOTE_MODE_ADVANCED:
                vz_set = chassis_vz;

                chassis->SetSpeed(chassis_vx, chassis_vy, vz_set);
                osDelay(1);
                chassis->SetPower(POWER_LIMIT_STATUS, referee->game_robot_status.chassis_power_limit,
                                  referee->power_heat_data.chassis_power,
                                  referee->power_heat_data.chassis_power_buffer);
                osDelay(1);
                vx_set = chassis_vx;
                vy_set = chassis_vy;
                vz_set = chassis_vz;
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
    can_bridge = new communication::CanBridge(can1, 0x51);
    chassis = new control::ChassisCanBridgeSender(can_bridge, 0x52);
    chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    chassis->Disable();
}
void kill_chassis() {
    chassis->Disable();
}