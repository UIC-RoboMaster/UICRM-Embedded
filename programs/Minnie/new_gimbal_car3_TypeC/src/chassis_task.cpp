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

// const float chassis_max_xy_speed = 4 * PI;
// const float chassis_max_t_speed = 12 * PI;
const float chassis_max_xy_speed = 8 * PI;
const float chassis_max_t_speed = 36 * PI;

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;
bool chassis_boost_flag = true;

float car_vx, car_vy, car_vt;

communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;

void debug_chassis_init() {
#ifdef CHASSIS_DEBUG
    print("CHASSIS_INIT_OK\n");
#endif
}

void debug_chassis(bool retprint, bool newline) {
    (void)retprint;
    (void)newline;
#ifdef CHASSIS_DEBUG
    chassis->debug_chassis_can_model(retprint, newline);
    chassis->debug_chassis_RCCH(0x74);
#endif
}

void chassis_ADVANCED() {
    const float ratio = 1.0f / 660.0f * PI * 40;
    const int ratio_vt = 50;
    chassis_vx = car_vx;
    chassis_vy = car_vy;
    chassis_vt = car_vt * ratio * ratio_vt;
}

void chassis_AUTOAIM() {
    chassis_vx = car_vx;
    chassis_vy = car_vy;
    chassis_vt = car_vt;
}

void chassis_FOLLOW(float chassis_yaw_diff) {
    // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
    const float angle_threshold = 0.02f;
    float chassis_vt_pid_error = chassis_yaw_diff;
    if (fabs(chassis_vt_pid_error) < angle_threshold) {
        chassis_vt_pid_error = 0;
    }

    static control::ConstrainedPID* chassis_vt_pid =
        new control::ConstrainedPID(12 / (2 * PI), 0, 8, 0.5, 8);
    float vt = chassis_vt_pid->ComputeOutput(chassis_vt_pid_error);
    if (chassis_vt_pid_error != 0)
        chassis_vt = vt;
}

void chassis_SPIN() {
    // 小陀螺模式，拨盘用来控制底盘加速度
    static float spin_speed = 2 * PI;
    spin_speed = spin_speed + car_vt * 0.1;
    // spin_speed = clip<float>(spin_speed, -1, 1);
    spin_speed = clip<float>(spin_speed, -4 * PI, 4 * PI);
    chassis_vt = spin_speed;
}

void chassis_PREPARE_HAND_MOVEMENT() {
    print("SITCK_SHIFT_CHASSIS: vx: %.2f, vy: %.2f, vt: %.2f\n", car_vx, car_vy, car_vt);
    const float ratio = 1.0f / 660.0f * 5 * PI;
    chassis_vx = car_vx * ratio;
    chassis_vy = car_vy * ratio;
    chassis_vt = car_vt * ratio;
}

void chassisTask(void* arg) {
    UNUSED(arg);
    kill_chassis();
    osDelay(1000);

    while (protect_wraning_flag) {
        osDelay(PROTECT_OS_DELAY);
    }

    while (remote_mode == REMOTE_MODE_KILL) {
        kill_chassis();
        osDelay(CHASSIS_OS_DELAY);
    }

    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    chassis->Enable();

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL ||
            !referee->game_robot_status.mains_power_chassis_output) {
            kill_chassis();
            while (remote_mode == REMOTE_MODE_KILL) {
                osDelay(CHASSIS_OS_DELAY + 2);
            }
            chassis->Enable();
            continue;
        }

        // todo: 遥控器与在线键盘控制底盘速度逻辑
        chassis_remote_mode();

        // 云台和底盘的角度差
        float chassis_yaw_diff = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        // 底盘以底盘自己为基准的运动速度
        float sin_yaw = arm_sin_f32(chassis_yaw_diff);
        float cos_yaw = arm_cos_f32(chassis_yaw_diff);
        chassis_vx = cos_yaw * car_vx + sin_yaw * car_vy;
        chassis_vy = -sin_yaw * car_vx + cos_yaw * car_vy;
        chassis_vt = 0;

        switch (remote_mode) {
            case REMOTE_MODE_ADVANCED:
                chassis_ADVANCED();
                break;

            case REMOTE_MODE_AUTOAIM:
                chassis_AUTOAIM();
                break;

            case REMOTE_MODE_FOLLOW:
                // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
                chassis_FOLLOW(chassis_yaw_diff);
                break;

            case REMOTE_MODE_SPIN:
                // 小陀螺模式，拨盘用来控制底盘加速度
                chassis_SPIN();
                break;

            case REMOTE_MODE_PREPARE_HAND_MOVEMENT:
                // 手动模式下，遥控器直接控制底盘速度
                chassis_PREPARE_HAND_MOVEMENT();
                break;

            default:
                break;
        }

        // 进行缩放
        chassis_vx *= chassis_max_xy_speed;
        chassis_vy *= chassis_max_xy_speed;
        chassis_vt *= chassis_max_t_speed;

        //        static const float move_ease_ratio = 0.5;
        //        static const float turn_ease_ratio = 0.9;
        //        static Ease chassis_ease_vx(0, move_ease_ratio);
        //        static Ease chassis_ease_vy(0, move_ease_ratio);
        //        static Ease chassis_ease_vt(0, turn_ease_ratio);
        //        chassis_vx = chassis_ease_vx.Calc(chassis_vx);
        //        chassis_vy = chassis_ease_vy.Calc(chassis_vy);
        //        chassis_vt = chassis_ease_vt.Calc(chassis_vt);

        chassis->SetSpeed(chassis_vx, chassis_vy, chassis_vt);
        osDelay(CHASSIS_OS_DELAY);
        chassis->SetPower(false, referee->game_robot_status.chassis_power_limit,
                          referee->power_heat_data.chassis_power,
                          referee->power_heat_data.chassis_power_buffer, false);
        osDelay(CHASSIS_OS_DELAY);
    }
}

void chassis_remote_mode() {
#ifdef DBUS_MODE
    remote::keyboard_t keyboard;
    if (dbus->IsOnline()) {
        keyboard = dbus->keyboard;
    } else if (refereerc->IsOnline()) {
        keyboard = refereerc->remote_control.keyboard;
    }

    // 以云台为基准的（整车的）运动速度，范围为[-1, 1]
    // float car_vx, car_vy, car_vt;
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
        updateCarVelocity(car_vx, car_vy, car_vt, 0.01);
    } else {
        // 使用键盘
        const float keyboard_speed = keyboard.bit.SHIFT ? 1 : 0.5;
        const float keyboard_spin_speed = 0.4;
        car_vx = (keyboard.bit.D - keyboard.bit.A) * keyboard_speed;
        car_vy = (keyboard.bit.W - keyboard.bit.S) * keyboard_speed;
        car_vt = (keyboard.bit.E - keyboard.bit.Q) * keyboard_spin_speed;
        updateCarVelocity(car_vx, car_vy, car_vt, 0.01, 0.1f);
    }
#endif
}

// 假设这是你的更新函数
void updateCarVelocity(float& car_vx, float& car_vy, float& car_vt, float deltaTime,
                       const float acceleration) {
    // 根据时间增量逐步增加速度
    car_vx += acceleration * deltaTime;
    car_vy += acceleration * deltaTime;
    car_vt += acceleration * deltaTime;
}

void Slow_start(float& in_val, float deltaTime, const float acceleration) {
    in_val += acceleration * deltaTime;
}

void init_chassis() {
    // 添加can bridge，注册本机ID
    can_bridge = new communication::CanBridge(can1, 0x51);
    // 添加can bridge的底盘控制器
    chassis = new control::ChassisCanBridgeSender(can_bridge, 0x52);
    // 设置底盘各目标的寄存器id
    chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    chassis->Disable();
    debug_chassis_init();
}

void kill_chassis() {
    chassis->Disable();
}