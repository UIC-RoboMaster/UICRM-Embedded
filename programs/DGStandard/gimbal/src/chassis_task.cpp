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
#include "remote_control.h"
osThreadId_t chassisTaskHandle;

// false：推杆以底盘车头为方向基准；true：恢复以枪口为方向基准的换算。
const bool chassis_gimbal_relative = false;

// 与实车可用的参考程序一致：满杆平移指令为 6 * PI，不再额外乘 0.5。
const float chassis_max_xy_speed = 6 * PI;
const float chassis_max_t_speed = 3 * PI;
// 每轮允许的速度指令变化量，越大则加减速越快。
const float chassis_xy_ramp_step = 0.12f;
const float chassis_turn_ramp_step = 0.06f;

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;
bool chassis_boost_flag = true;
float car_vx, car_vy, car_vt;

diff<float> car_vx_diff, car_vy_diff, car_vt_diff;

communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;
bool chassis_control_enabled = false;

void chassisTask(void* arg) {
    UNUSED(arg);
    kill_chassis();
    osDelay(1000);
    Ease chassis_ease_vx(0, chassis_xy_ramp_step);
    Ease chassis_ease_vy(0, chassis_xy_ramp_step);
    Ease chassis_ease_vt(0, chassis_turn_ramp_step);

    while (true) {
        const bool enabled = dbus->IsOnline() &&
                             (dbus->swr == remote::UP || dbus->swr == remote::MID) &&
                             remote_mode == REMOTE_MODE_FOLLOW &&
                             imu->CaliDone() && yaw_motor->IsOnline();
        if (!enabled) {
            kill_chassis();
            chassis_ease_vx = Ease(0, chassis_xy_ramp_step);
            chassis_ease_vy = Ease(0, chassis_xy_ramp_step);
            chassis_ease_vt = Ease(0, chassis_turn_ramp_step);
            osDelay(CHASSIS_OS_DELAY);
            continue;
        }
        if (!chassis_control_enabled) {
            chassis->Enable();
            chassis_control_enabled = true;
        }

        const remote::keyboard_t keyboard = dbus->keyboard;

        // 摇杆输入；方向基准由 chassis_gimbal_relative 选择。
        //        if (keyboard.bit.X) {
        //            // 刹车
        //            car_vx = 0;
        //            car_vy = 0;
        //            car_vt = 0;
        //        } else
        if (dbus->ch0 || dbus->ch1 || dbus->ch2 || dbus->ch3 || dbus->ch4) {
            // 优先使用遥控器
            const float speed_scale = 1.0f;
            const int rocker_deadband = 10;
            car_vx = dgstandard::NormalizeRocker(dbus->ch0, dbus->ROCKER_MAX, rocker_deadband) * speed_scale;
            car_vy = dgstandard::NormalizeRocker(dbus->ch1, dbus->ROCKER_MAX, rocker_deadband) * speed_scale;
            car_vt = dgstandard::NormalizeRocker(dbus->ch4, dbus->ROCKER_MAX, rocker_deadband) * speed_scale;
        } else {
            // 使用键盘
            const float keyboard_speed = keyboard.bit.SHIFT ? 1 : 0.5;
            const float keyboard_spin_speed = 1;
            car_vx = (keyboard.bit.D - keyboard.bit.A) * keyboard_speed;
            car_vy = (keyboard.bit.W - keyboard.bit.S) * keyboard_speed;
            car_vt = (keyboard.bit.E - keyboard.bit.Q) * keyboard_spin_speed;
        }

        static const float car_move_ease_ratio = 0.6;
        static const float car_turn_ease_ratio = 0.9;
        static Ease car_ease_vx(0, car_move_ease_ratio);
        static Ease car_ease_vy(0, car_move_ease_ratio);
        static Ease car_ease_vt(0, car_turn_ease_ratio);
        car_vx = car_ease_vx.Calc(car_vx);
        car_vy = car_ease_vy.Calc(car_vy);
        car_vt = car_ease_vt.Calc(car_vt);

        car_vx_diff.calc(car_vx);
        car_vy_diff.calc(car_vy);
        car_vt_diff.calc(car_vt);

        // 用实际 Yaw 编码器角度转换平移方向，避免混入 IMU 世界坐标。
        const float A = wrapc<float>(yaw_motor->GetTheta() - gimbal_param->yaw_offset_, -PI, PI);
        const float chassis_target_diff = chassis_gimbal_relative ? -A : 0;

        // 底盘以底盘自己为基准的运动速度
        float sin_yaw = arm_sin_f32(chassis_target_diff);
        float cos_yaw = arm_cos_f32(chassis_target_diff);
        chassis_vx = cos_yaw * car_vx + sin_yaw * car_vy;
        chassis_vy = -sin_yaw * car_vx + cos_yaw * car_vy;
        // 当前 Yaw 保持相对底盘的编码器角度，自动跟随会持续追逐这个固定偏角。
        // 先用 CH4 拨轮手动转向；恢复 IMU 稳定后再启用底盘自动跟随。
        chassis_vt = car_vt;

        // 进行缩放
        chassis_vx *= chassis_max_xy_speed;
        chassis_vy *= chassis_max_xy_speed;
        chassis_vt *= chassis_max_t_speed;

        chassis_vx = chassis_ease_vx.Calc(chassis_vx);
        chassis_vy = chassis_ease_vy.Calc(chassis_vy);
        chassis_vt = chassis_ease_vt.Calc(chassis_vt);

        chassis->SetSpeed(chassis_vx, chassis_vy, chassis_vt);
        osDelay(1);

#ifdef HAS_REFEREE
        uint8_t buffer_percent = referee->power_heat_data.chassis_power_buffer * 100 / 60;
        uint8_t max_watt = referee->game_robot_status.chassis_power_limit;
#else
        uint8_t buffer_percent = 50;
        uint8_t max_watt = 100;
#endif
        // 如果传输的电压为0，底盘不会保存这个值，随后底盘的chassis_task会采样自己的电压值并更新。
        chassis->UpdatePower(true, max_watt, 0, buffer_percent);
        osDelay(CHASSIS_OS_DELAY);
    }
}

void init_chassis() {
    // 添加can bridge，注册本机ID
    can_bridge = new communication::CanBridge(can1, 0x51);
    // 添加can bridge的底盘控制器
    chassis = new control::ChassisCanBridgeSender(can_bridge, 0x52);
    // 设置底盘各目标的寄存器id
    chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    chassis->Disable();
}
void kill_chassis() {
    chassis_control_enabled = false;
    chassis_vx = chassis_vy = chassis_vt = 0;
    chassis->Disable();
}
