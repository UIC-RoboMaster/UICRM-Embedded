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

#include "gimbal_task.h"

#include "chassis_task.h"
#include "config.h"
#include "dbus_package.h"
#include "minipc_task.h"

osThreadId_t gimbalTaskHandle;

driver::MotorCANBase* pitch_motor = nullptr;
driver::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
INS_Angle_t INS_Angle;

control::gimbal_t gimbal_data;

void check_kill();

void gimbalTask(void* arg) {
    UNUSED(arg);
    // 任务启动时先关掉两个电机，然后等待遥控器连接
    pitch_motor->Disable();
    yaw_motor->Disable();
    osDelay(1500);

    // 遥控器连接后等待一段时间，等云台完全复位
    int i;
    for (i = 0; i < 5000; i++) {
        check_kill();
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
    }

    // 云台复位完成后，播放一段音乐，代表开始校准陀螺仪
    // 校准陀螺仪会对陀螺仪进行2000次的读取，然后取平均值作为校准值
    // buzzer->SingTone(bsp::BuzzerNote::La6M);
    Buzzer_Sing(SingCaliStart);
    //    reset_yaw();
    ahrs->Cailbrate();
    i = 0;
    while (!ahrs->IsCailbrated()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
        ++i;
    }
    // 校准完成后播放一段音乐
    Buzzer_Sing(SingCaliDone);
    osDelay(100);

    // 初始化当前陀螺仪角度、遥控器输入转换的角度、目标角度
    float pitch_ratio, yaw_ratio;
    INS_Angle.pitch = ahrs->INS_angle[2];
    INS_Angle.yaw = ahrs->INS_angle[0];
    // IMU Yaw 展开（避免跨 ±π 时参考角跳变）
    static float yaw_unwrap_offset = 0.0f;
    static float yaw_last_raw = ahrs->INS_angle[0];

    //    pitch_curr = witimu->INS_angle[0];
    //    yaw_curr = wrap<float>(witimu->INS_angle[2]-yaw_offset, -PI, PI);
    float pitch_target = 0, yaw_target = 0;
        // 绝对目标（基于连续 IMU 角）用于环绕模式避免跨 ±π 跳变
        static bool abs_target_inited = false;
        static float pitch_abs_target = 0.0f;
        static float yaw_abs_target = 0.0f;

    while (true) {
        // 如果遥控器处于关闭状态，关闭两个电机
        check_kill();

        // 获取当前陀螺仪角度
        INS_Angle.pitch = ahrs->INS_angle[2];
        INS_Angle.yaw = ahrs->INS_angle[0];
        // 生成连续的 IMU Yaw
        float yaw_raw = INS_Angle.yaw;
        float dy = yaw_raw - yaw_last_raw;
        if (dy > PI)       yaw_unwrap_offset -= 2 * PI;
        else if (dy < -PI) yaw_unwrap_offset += 2 * PI;
        float yaw_continuous = yaw_raw + yaw_unwrap_offset;
        yaw_last_raw = yaw_raw;

        // 如果遥控器处于开机状态，优先使用遥控器输入，否则使用裁判系统图传输入
        const float mouse_ratio = 1;
        const float remote_ratio = 0.005;
        if (dbus->IsOnline()) {
            if (dbus->mouse.x != 0 || dbus->mouse.y != 0) {
                pitch_ratio = (float)dbus->mouse.y / mouse_xy_max * mouse_ratio;
                yaw_ratio = (float)dbus->mouse.x / mouse_xy_max * mouse_ratio;
            } else {
                pitch_ratio = (float)dbus->ch3 / dbus->ROCKER_MAX * remote_ratio;
                yaw_ratio = (float)dbus->ch2 / dbus->ROCKER_MAX * remote_ratio;
            }
        } else if (refereerc->IsOnline()) {
            pitch_ratio = -refereerc->remote_control.mouse.y / mouse_xy_max * mouse_ratio;
            yaw_ratio = -refereerc->remote_control.mouse.x / mouse_xy_max * mouse_ratio;
        } else {
            pitch_ratio = 0;
            yaw_ratio = 0;
        }

        // 根据遥控器输入计算目标角度，并且进行限幅
        // pitch_target =
        //     clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        // yaw_target = wrap<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);
        //
        // pitch_diff = clip<float>(pitch_target, -PI, PI);
        // yaw_diff = wrap<float>(yaw_target, -PI, PI);

            // 初始化一次绝对目标为当前 IMU 连续角
            if (!abs_target_inited) {
                pitch_abs_target = INS_Angle.pitch;
                yaw_abs_target = yaw_continuous;
                abs_target_inited = true;
            }

            // 根据遥控器输入更新绝对目标（每周期增量积分）
            // PITCH 受物理最大范围限制
            pitch_abs_target += pitch_ratio;
            pitch_abs_target = clip<float>(pitch_abs_target, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);

            if (gimbal_param->yaw_circle_) {
                // YAW 环绕：允许多圈，直接积分
                yaw_abs_target += yaw_ratio;
            } else {
                // YAW 非环绕：限制在设定范围
                yaw_abs_target += yaw_ratio;
                yaw_abs_target = wrap<float>(yaw_abs_target, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);
            }

            // 计算用于 UI 的误差（目标-当前），避免显示跳变
            pitch_diff = clip<float>(pitch_abs_target - INS_Angle.pitch, -PI, PI);
            yaw_diff = wrap<float>(yaw_abs_target - yaw_continuous, -PI, PI);

        pitch_target = clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        if (gimbal_param->yaw_circle_) {
            // 环绕模式：使用增量控制，不做 ±π 包裹
            yaw_target = yaw_ratio;
            yaw_diff = yaw_target;
        } else {
            // 非环绕模式：按原逻辑限幅
            yaw_target = wrap<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);
            yaw_diff = wrap<float>(yaw_target, -PI, PI);
        }
        pitch_diff = clip<float>(pitch_target, -PI, PI);

        // 根据运动模式选择不同的控制方式
        const float ratio = 0.1875;
        float speed_offset = chassis_vt * ratio;
        yaw_motor->SetSpeedOffset(speed_offset);
        if (is_autoaim && minipc->IsOnline() && minipc->target_angle.target_robot_id != 0) {
            gimbal->TargetAbs(minipc->target_angle.target_pitch, -minipc->target_angle.target_yaw);
            gimbal->UpdateIMU(INS_Angle.pitch, yaw_continuous);
        } else {
            switch (remote_mode) {
                case REMOTE_MODE_SPIN:
                case REMOTE_MODE_FOLLOW:
                        // 如果是跟随模式或者旋转模式，使用绝对目标避免跨 ±π 抖动
                        gimbal->TargetAbs(pitch_abs_target, yaw_abs_target);
                    gimbal->UpdateIMU(INS_Angle.pitch, yaw_continuous);
                    break;
                case REMOTE_MODE_ADVANCED:
                    gimbal->TargetRel(pitch_diff, yaw_diff);
                    gimbal->Update();
                    break;
                case REMOTE_MODE_AUTOAIM:
                    if (static_cast<float>(minipc->target_angle.accuracy) < 60.0f)
                        break;
                    gimbal->TargetAbs(minipc->target_angle.target_pitch,
                                      -minipc->target_angle.target_yaw);
                    gimbal->UpdateIMU(INS_Angle.pitch, yaw_continuous);
                    break;
                default:
                    break;
            }
        }

        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal() {
    // 云台需要使用两个6020电机，并且进行角度环和速度环双环控制。此时我们需要初始化两个电机对象和四个PID对象，并且将四个PID对象分别绑定到两个电机对象上。

    /**
     * pitch motor
     */
    pitch_motor = new driver::Motor6020(GIMBAL_CAN, PITCH_MOTOR_RX_ID, PITCH_MOTOR_TX_ID);
    pitch_motor->SetTransmissionRatio(1);

    //
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 12,
        .ki = 0,
        .kd = 30,
        .max_out = 6 * PI,  // 最高旋转速度
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);

    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 4096,
        .ki = 0,
        .kd = 1200,
        .max_out = 16384,  // 最大电流输出，参考说明书
        .max_iout = 4000,
        .deadband = 0,                          // 死区
        .A = 1.5 * PI,                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    // 给电机启动角度环和速度环，并且这是一个绝对角度电机，需要启动绝对角度模式
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::ANGEL_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    /**
     * yaw motor
     */
    yaw_motor = new driver::Motor6020(GIMBAL_CAN, YAW_MOTOR_RX_ID, YAW_MOTOR_TX_ID);
    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_motor_theta_pid_init = {
        .kp = 2,
        .ki = 0,
        .kd = 0,
        .max_out = 4 * PI,  // 最高旋转速度
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.16,          // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 1000,
        .ki = 0,
        .kd = 0,
        .max_out = 16384,  // 最大电流输出，参考说明书
        .max_iout = 2000,
        .deadband = 0,                            // 死区
        .A = 0.5 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 0.5 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.03,     // 输出滤波系数
        .derivative_filtering_coefficient = 0.1,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    yaw_motor->ReInitPID(yaw_motor_omega_pid_init, driver::MotorCANBase::ANGEL_LOOP_CONTROL );
    // 给电机启动角度环和速度环，并且这是一个绝对角度电机，需要启动绝对角度模式
    // yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
    //                    driver::MotorCANBase::ANGLE_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    // 环绕模式下禁用单圈绝对角度模式，避免跨 ±π 折返
    if (gimbal_init_data.yaw_circle_) {
        yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                           driver::MotorCANBase::ANGEL_LOOP_CONTROL);
    } else {
        yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                           driver::MotorCANBase::ANGEL_LOOP_CONTROL |
                           driver::MotorCANBase::ABSOLUTE);
    }

    // 初始化云台对象
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}
void check_kill() {
    if (remote_mode == REMOTE_MODE_KILL) {
        yaw_motor->Disable();
        pitch_motor->Disable();
        steering_motor->Disable();
        while (remote_mode == REMOTE_MODE_KILL)
            osDelay(1);
    }
    yaw_motor->Enable();
    pitch_motor->Enable();
    steering_motor->Enable();
}