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

float chassis_vx = 0;
float chassis_vy = 0;
bool chassis_boost_flag = true;
const float max_speed = 660;
const float max_speed_boost = 1320;

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

    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    // float last_speed = 0;
    float offset_yaw = 0;
    float spin_speed = 350;
    float manual_mode_yaw_pid_args[3] = {200, 0.5, 0};
    float manual_mode_yaw_pid_max_iout = 100;
    float manual_mode_yaw_pid_max_out = 350;
    control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
        manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
    manual_mode_pid->Reset();
    float yaw_pid_error = 0;
    float manual_mode_pid_output = 0;
    float current_speed_offset = max_speed;
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
    BoolEdgeDetector* ch0_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch1_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch2_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch3_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch4_edge = new BoolEdgeDetector(false);

    chassis->Enable();

    const float ratio = 1.0f / 660.0f * 12 * PI;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            while (remote_mode == REMOTE_MODE_KILL) {
                osDelay(CHASSIS_OS_DELAY + 2);
            }
            chassis->Enable();
            continue;
        }

        remote::keyboard_t keyboard;
        if (selftest.dbus) {
            keyboard = dbus->keyboard;
        } else if (selftest.refereerc) {
                keyboard = refereerc->remote_control.keyboard;
        }

        float yaw_motor_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        float sin_yaw = arm_sin_f32(yaw_motor_angle);
        float cos_yaw = arm_cos_f32(yaw_motor_angle);


        ch0_edge->input(dbus->ch0 != 0);
        ch1_edge->input(dbus->ch1 != 0);
        ch2_edge->input(dbus->ch2 != 0);
        ch3_edge->input(dbus->ch3 != 0);
        ch4_edge->input(dbus->ch4 != 0);

        w_edge->input(keyboard.bit.W);
        s_edge->input(keyboard.bit.S);
        a_edge->input(keyboard.bit.A);
        d_edge->input(keyboard.bit.D);
        shift_edge->input(keyboard.bit.SHIFT);
        q_edge->input(keyboard.bit.Q);
        e_edge->input(keyboard.bit.E);
        x_edge->input(keyboard.bit.X);


        float speed = max_speed;
        chassis_boost_flag = keyboard.bit.SHIFT;
        if (chassis_boost_flag) {
            speed = max_speed_boost;
        }

        float curr_vx, curr_vy, curr_vt;

        // 刹车功能
        if (keyboard.bit.X) {
            curr_vx = 0;
            curr_vy = 0;
            curr_vt = 0;
        } else if (dbus->ch0 || dbus->ch1 || dbus->ch2 || dbus->ch3 || dbus->ch4) {
            curr_vx = dbus->ch0;
            curr_vy = dbus->ch1;
            curr_vt = dbus->ch2;
        } else {
            curr_vx = (keyboard.bit.D - keyboard.bit.A) * speed;
            curr_vy = (keyboard.bit.W - keyboard.bit.S) * speed;
            curr_vt = (keyboard.bit.E - keyboard.bit.Q) * speed;
        }

        float vx_set_org = 0, vy_set_org = 0;
        if (ch0_edge->get()) {
            // 遥控器有输入
            vx_set_org = dbus->ch0;
        } else if (ch0_edge->negEdge() || x_edge->posEdge()) {
            // 刹车
            vx_set_org = 0;
            vx_ramp->SetCurrent(0);
        } else if (d_edge->get()) {
            vx_set_org = vx_ramp->Calc(current_speed_offset);
        } else if (a_edge->get()) {
            vx_set_org = vx_ramp->Calc(-current_speed_offset);
        } else {
            // 键盘、遥控器都没有输入
            if (vx_set_org > 0) {
                vx_set_org = vx_ramp->Calc(-current_speed_offset);
            } else if (vx_set_org < 0) {
                vx_set_org = vx_ramp->Calc(current_speed_offset);
            }
        }
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
        chassis_vx = vx_set_org;
        chassis_vy = vy_set_org;
        chassis_vz = offset_yaw;
        float vx_set = cos_yaw * vx_set_org + sin_yaw * vy_set_org;
        float vy_set = -sin_yaw * vx_set_org + cos_yaw * vy_set_org;
        float vz_set = 0;
        switch (remote_mode) {
            case REMOTE_MODE_FOLLOW:
                yaw_pid_error = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
                if (fabs(yaw_pid_error) < 0.01f) {
                    yaw_pid_error = 0;
                }
                manual_mode_pid_output = manual_mode_pid->ComputeOutput(yaw_pid_error);
                chassis->SetSpeed(vx_set * ratio, vy_set * ratio, manual_mode_pid_output * ratio);
                osDelay(1);
                break;
            case REMOTE_MODE_SPIN:

                if (offset_yaw != 0) {
                    spin_speed = spin_speed + offset_yaw;
                    offset_yaw = 0;
                    spin_speed = clip<float>(spin_speed, -660, 660);
                }
                vz_set = spin_speed;
                chassis->SetSpeed(vx_set * ratio, vy_set * ratio, vz_set * ratio);
                osDelay(1);
                break;
            case REMOTE_MODE_ADVANCED:
                vz_set = offset_yaw;
                if (offset_yaw != 0) {
                    spin_speed = spin_speed + offset_yaw;
                    offset_yaw = 0;
                    spin_speed = clip<float>(spin_speed, -660, 660);
                }
                chassis->SetSpeed(vx_set_org * ratio, vy_set_org * ratio, vz_set * ratio);
                osDelay(1);
                break;
            default:
                // Not Support
                kill_chassis();
        }
        chassis->SetPower(false, referee->game_robot_status.chassis_power_limit,
                          referee->power_heat_data.chassis_power,
                          referee->power_heat_data.chassis_power_buffer);
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