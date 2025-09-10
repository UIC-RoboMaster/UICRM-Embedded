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

#include "dbus_package.h"

osThreadId_t gimbalTaskHandle;

driver::Motor6020* pitch_motor = nullptr;
driver::Motor6020* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_target = 0, yaw_target = 0;
float pitch_diff, yaw_diff;
INS_Angle_t INS_Angle;

control::gimbal_t gimbal_data;

bool LAST_REMOTE_MODE = false;

template <typename T>
T map(T value, T in_min, T in_max, T out_min, T out_max) {
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

void debug_gimbal_init() {
#ifdef GINBAL_DEBUG
    print("GIMBAL_INIT_OK\n");
#endif
}

void debug_gimbal(bool newline) {
    (void)newline;
#ifdef GINBAL_DEBUG
    print("Gimbal_pitch: %.2f, Gimbal_yaw: %.2f", INS_Angle.pitch, INS_Angle.yaw);
    print("Gimbal_pitch_diff: %.2f, Gimbal_yaw_diff: %.2f\n", pitch_diff, yaw_diff);
    if (newline) {
        print("\r\n");
    }
#endif
}

void check_kill();

void gimbalTask(void* arg) {
    UNUSED(arg);
    // 任务启动时先关掉两个电机，然后等待遥控器连接
    pitch_motor->Disable();
    yaw_motor->Disable();
    osDelay(1500);

    while (protect_wraning_flag) {
        osDelay(PROTECT_OS_DELAY);
    }

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
    Buzzer_Sing(SingCaliStart);

    ahrs->Cailbrate();
    i = 0;
    while (!ahrs->IsCailbrated()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
        ++i;
    }
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    osDelay(10);

    // 校准完成后播放一段音乐
    Buzzer_Sing(SingCaliDone);
    osDelay(100);

    //    pitch_curr = witimu->INS_angle[0];
    //    yaw_curr = wrap<float>(witimu->INS_angle[2]-yaw_offset, -PI, PI);

    while (true) {
        // 如果遥控器处于关闭状态，关闭两个电机
        check_kill();
        INS_Angle.pitch = ahrs->INS_angle[2];
        INS_Angle.yaw = ahrs->INS_angle[0];

        auto [pitch_ratio, yaw_ratio] = gimbal_remote_mode();

        // 根据遥控器输入计算目标角度，并且进行限幅
        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = wrap<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, PI, PI);

        if (abs(yaw_diff) < 0.00001)
            yaw_diff = 0.0000000f;

        // todo 超级补丁，完全修复需要重构云台更新逻辑，现在的逻辑是yaw_target,
        // pitch_target实际作为作为diff，云台实际上加减target而非diff。这里作为临时限位。
        float backVal = 0.0001;
        float maxYaw = PI;
        float absYaw = abs(INS_Angle.yaw);
        if (absYaw > maxYaw)
            yaw_diff = -INS_Angle.yaw / absYaw * backVal;

        // 根据运动模式选择不同的控制方式
        // const float ratio = 0.1875;
        // float speed_offset = 1 * ratio;
        // yaw_motor->SetSpeedOffset(speed_offset);

        switch (remote_mode) {
            case REMOTE_MODE_FOLLOW:
                // 如果是跟随模式或者旋转模式，将IMU作为参考系
                gimbal->TargetRel(-pitch_diff, -yaw_diff);
                gimbal->UpdateIMU(INS_Angle.pitch, INS_Angle.yaw);
                break;

            case REMOTE_MODE_ADVANCED:
                // 如果是高级模式，将电机获取的云台当前角度作为参考系
                gimbal->TargetRel(-pitch_diff, -yaw_diff);
                gimbal->Update();
                break;

            case REMOTE_MODE_PREPARE_HAND_MOVEMENT:
                // 遥控器手动控制模式，将电机获取的云台当前角度作为参考系，直接通过遥控控制

                gimbal->TargetRel(pitch_diff, yaw_diff);
                print("pitch_motor: %.2f pitch_diff: %.2f\r\n", map<float>(pitch_motor->GetTheta(), 0, 2 * PI, -PI, PI));
                gimbal->Update();
                break;

            default:
                break;
        }

        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal() {
    // 云台需要使用两个6020电机，并且进行角度环和速度环双环控制。此时我们需要初始化两个电机对象和四个PID对象，并且将四个PID对象分别绑定到两个电机对象上。

    /**
     * pitch motor
     */
    pitch_motor = new driver::Motor6020(can1, 0x208, 0x1FF);
    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 35,
        .ki = 20,
        .kd = 800,
        .max_out = 4 * PI,  // 最高旋转速度
        .max_iout = 0,
        .deadband = 0,                                     // 死区
        .A = 0,                                            // 变速积分所能达到的最大值为A+B
        .B = 0,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,               // 输出滤波系数
        .derivative_filtering_coefficient = 0,             // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |  // 积分限幅
                control::ConstrainedPID::OutputFilter      // 输出滤波
    };

    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 2000,
        .ki = 25,
        .kd = 2000,
        .max_out = 16384,  // 最大电流输出，参考说明书
        .max_iout = 2000,
        .deadband = 0,                                           // 死区
        .A = 0.5 * PI,                                           // 变速积分所能达到的最大值为A+B
        .B = 0.5 * PI,                                           // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                     // 输出滤波系数
        .derivative_filtering_coefficient = 0,                   // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    // 给电机启动角度环和速度环，并且这是一个绝对角度电机，需要启动绝对角度模式
    pitch_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                         driver::MotorCANBase::ABSOLUTE);

    /**
     * yaw motor
     */
    yaw_motor = new driver::Motor6020(can1, 0x206, 0x1FF);
    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_motor_theta_pid_init = {
        .kp = 25,
        .ki = 1,
        .kd = 1000,
        .max_out = 4 * PI,  // 最高旋转速度
        .max_iout = 0,
        .deadband = 0,                                     // 死区
        .A = 0,                                            // 变速积分所能达到的最大值为A+B
        .B = 0,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,               // 输出滤波系数
        .derivative_filtering_coefficient = 0,             // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |  // 积分限幅
                control::ConstrainedPID::OutputFilter      // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 1200,
        .ki = 5,
        .kd = 2000,
        .max_out = 16384,  // 最大电流输出，参考说明书
        .max_iout = 2000,
        .deadband = 0,                                           // 死区
        .A = 0.5 * PI,                                           // 变速积分所能达到的最大值为A+B
        .B = 0.5 * PI,                                           // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                     // 输出滤波系数
        .derivative_filtering_coefficient = 0.1,                 // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    yaw_motor->ReInitPID(yaw_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    // 给电机启动角度环和速度环，并且这是一个绝对角度电机，需要启动绝对角度模式
    yaw_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                       driver::MotorCANBase::ABSOLUTE);

    // 初始化云台对象
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
    debug_gimbal_init();
}

Gimbal_Targe_data gimbal_remote_mode() {
#ifdef DBUS_MODE
    // 如果遥控器处于开机状态，优先使用遥控器输入，否则使用裁判系统图传输入
    Gimbal_Targe_data Gimbal_Targe;
    const float mouse_ratio = 1;
    const float remote_ratio = 0.001;
    if (dbus->IsOnline()) {
        if (dbus->mouse.x != 0 || dbus->mouse.y != 0) {
            Gimbal_Targe.pitch_ratio_targe = (float)dbus->mouse.y / mouse_xy_max * mouse_ratio;
            Gimbal_Targe.yaw_ratio_targe = (float)dbus->mouse.x / mouse_xy_max * mouse_ratio;
        } else {
            Gimbal_Targe.pitch_ratio_targe = (float)dbus->ch3 / dbus->ROCKER_MAX * remote_ratio;
            Gimbal_Targe.yaw_ratio_targe = (float)dbus->ch0 / dbus->ROCKER_MAX * remote_ratio;
        }
    } else {
        Gimbal_Targe.pitch_ratio_targe = 0;
        Gimbal_Targe.yaw_ratio_targe = 0;
    }
#endif

    return Gimbal_Targe;
}

void check_kill() {
    if (remote_mode == REMOTE_MODE_KILL) {
        yaw_motor->Disable();
        pitch_motor->Disable();
        while (remote_mode == REMOTE_MODE_KILL)
            osDelay(1);
    }
    yaw_motor->Enable();
    pitch_motor->Enable();
}