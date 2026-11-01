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
#include "chassis_control.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "yaw_follow_control.h"
osThreadId_t chassisTaskHandle;

// FOLLOW 追赶枪口，SPIN 固定转动；两种模式的平移均按枪口方向换算。
const bool chassis_gimbal_relative = true;

// 平移满杆速度；保留旧旋转比例供外部兼容，FOLLOW/SPIN 不再乘该比例。
const float chassis_max_xy_speed = 10 * PI;
const float chassis_max_t_speed = 5 * PI;
// FOLLOW/SPIN 的最终旋转指令及斜坡统一由 chassis_control.h 计算。

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;
float chassis_follow_error = 0;
const char* chassis_turn_phase = "OFF";
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
    dgstandard::TranslationRamp translation_ramp;
    dgstandard::ChassisTurnControl turn_control;
    const float tick_seconds = 1.0f / osKernelGetTickFreq();
    uint32_t previous_tick = osKernelGetTickCount();

    while (true) {
        const uint32_t now = osKernelGetTickCount();
        const float dt = clip<float>((now - previous_tick) * tick_seconds, 0, 0.02f);
        previous_tick = now;
        const RemoteMode mode = remote_mode;
        const bool enabled = dbus->IsOnline() &&
                             (dbus->swr == remote::UP || dbus->swr == remote::MID) &&
                             dgstandard::IsActiveRemoteMode(mode) &&
                             imu->CaliDone() && imu_attitude_fresh() && yaw_motor->IsOnline() && yaw_world_ready;
        if (!enabled) {
            kill_chassis();
            translation_ramp.Reset();
            car_vx = car_vy = car_vt = 0;
            turn_control.Reset();
            osDelay(CHASSIS_OS_DELAY);
            continue;
        }
        if (!chassis_control_enabled) {
            chassis->Enable();
            chassis_control_enabled = true;
        }

        const remote::keyboard_t keyboard = dbus->keyboard;

        // 摇杆平移以枪口为基准。
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
            car_vx = dgstandard::NormalizeRocker(
                dbus->ch0, dbus->ROCKER_MAX, dgstandard::kTranslationRockerDeadband) * speed_scale;
            car_vy = dgstandard::NormalizeRocker(
                dbus->ch1, dbus->ROCKER_MAX, dgstandard::kTranslationRockerDeadband) * speed_scale;
            car_vt = dgstandard::NormalizeRocker(dbus->ch4, dbus->ROCKER_MAX, rocker_deadband) * speed_scale;
        } else {
            // 使用键盘
            const float keyboard_speed = keyboard.bit.SHIFT ? 1 : 0.5;
            const float keyboard_spin_speed = 1;
            car_vx = (keyboard.bit.D - keyboard.bit.A) * keyboard_speed;
            car_vy = (keyboard.bit.W - keyboard.bit.S) * keyboard_speed;
            car_vt = (keyboard.bit.E - keyboard.bit.Q) * keyboard_spin_speed;
        }

        static const float car_turn_ease_ratio = 0.9;
        static Ease car_ease_vt(0, car_turn_ease_ratio);
        // 整体平滑枪口坐标下的速度，不能在方向换算后分别追赶 vx/vy。
        // 输入尚未乘满杆速度，因此加速度也换算为归一化单位。
        const auto gun_velocity = translation_ramp.Update(
            {car_vx, car_vy}, dt, dgstandard::kTranslationAcceleration / chassis_max_xy_speed);
        car_vx = gun_velocity.x;
        car_vy = gun_velocity.y;
        car_vt = car_ease_vt.Calc(car_vt);

        car_vx_diff.calc(car_vx);
        car_vy_diff.calc(car_vy);
        car_vt_diff.calc(car_vt);

        // 枪相对底盘的夹角。日志结合手转标定表明：正旋转指令使底盘
        // 沿编码器正方向转，因此用正夹角追赶。
        chassis_follow_error = wrapc<float>(yaw_motor->GetTheta() - gimbal_param->yaw_offset_, -PI, PI);

        // 按当前夹角立即换算，底盘自转时不滞留旧坐标中的速度。
        const auto body_velocity = dgstandard::RotateVelocity(
            gun_velocity, arm_sin_f32(chassis_follow_error), arm_cos_f32(chassis_follow_error));
        chassis_vx = body_velocity.x;
        chassis_vy = body_velocity.y;
        // 旋转已是最终指令单位，不再叠乘 chassis_max_t_speed；SPIN 不使用 CH4。
        chassis_vt = turn_control.Update(mode, chassis_follow_error, dt);
        chassis_turn_phase = turn_control.Phase();

        // 进行缩放
        chassis_vx *= chassis_max_xy_speed;
        chassis_vy *= chassis_max_xy_speed;

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
    chassis_follow_error = 0;
    chassis_turn_phase = "OFF";
    chassis->Disable();
}
