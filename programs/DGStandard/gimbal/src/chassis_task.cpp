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

    chassis->Enable();

    const float ratio = 1.0f / 660.0f * 12 * PI;
    float chassis_vt_pid_args[3] = {200, 0.5, 0};
    float chassis_vt_pid_max_iout = 100;
    float chassis_vt_pid_max_out = 350;
    control::ConstrainedPID* chassis_vt_pid = new control::ConstrainedPID(
        chassis_vt_pid_args, chassis_vt_pid_max_iout, chassis_vt_pid_max_out);

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

        // 底盘以云台为基准的（整车的）运动速度
        float car_vx, car_vy, car_vt;
        if (keyboard.bit.X) {
            // 刹车
            car_vx = 0;
            car_vy = 0;
            car_vt = 0;
        } else if (dbus->ch0 || dbus->ch1 || dbus->ch2 || dbus->ch3 || dbus->ch4) {
            // 优先使用遥控器
            car_vx = dbus->ch0;
            car_vy = dbus->ch1;
            car_vt = dbus->ch4;
        } else {
            // 使用键盘
            const float keyboard_speed = keyboard.bit.SHIFT ? 1320 : 660;
            const float keyboard_spin_speed = 10;
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
        float chassis_vt = 0;

        if (remote_mode == REMOTE_MODE_MANUAL)
        {
            // 手动模式下，底盘速度直接等于整车速度
            chassis_vt = car_vt;
        }

        if (remote_mode == REMOTE_MODE_FOLLOW)
        {
            // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
            const float angle_threshold = 0.01f;
            float chassis_vt_pid_error = chassis_yaw_diff;
            if (fabs(chassis_vt_pid_error) < angle_threshold) {
                chassis_vt_pid_error = 0;
            }
            chassis_vt += chassis_vt_pid->ComputeOutput(chassis_vt_pid_error);
        }
        if (remote_mode == REMOTE_MODE_SPIN)
        {
            // t轴用来控制底盘恒定旋转速度的增减
            static float spin_speed = 660;
            spin_speed = spin_speed + car_vt;
            spin_speed = clip<float>(spin_speed, -660, 660);
            chassis_vt = spin_speed;
        }

        chassis_vx*=ratio;
        chassis_vy*=ratio;
        chassis_vt*=ratio;

        chassis->SetSpeed(chassis_vx, chassis_vy, chassis_vt);
        osDelay(CHASSIS_OS_DELAY);
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