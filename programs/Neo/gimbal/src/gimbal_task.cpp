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
#include "minipc_task.h"

osThreadId_t gimbalTaskHandle;

driver::Motor6020* pitch_motor = nullptr;
driver::Motor6020* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
void gimbalTask(void* arg) {
    UNUSED(arg);

    pitch_motor->Disable();
    yaw_motor->Disable();
    osDelay(100);

    osDelay(1500);
    while (remote_mode == REMOTE_MODE_KILL) {
        kill_gimbal();
        osDelay(GIMBAL_OS_DELAY);
    }
    int i = 0;
    while (i < 5000 || !imu->DataReady()) {
        while (remote_mode == REMOTE_MODE_KILL) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
        }
        if (!pitch_motor->IsEnable())
            pitch_motor->Enable();
        if (!yaw_motor->IsEnable())
            yaw_motor->Enable();

        gimbal->TargetAbs(0, 0);
        gimbal->Update();

        osDelay(GIMBAL_OS_DELAY);
        ++i;
    }

    // 云台复位完成后，播放一段音乐，代表开始校准陀螺仪
    // 校准陀螺仪会对陀螺仪进行2000次的读取，然后取平均值作为校准值
    // buzzer->SingTone(bsp::BuzzerNote::La6M);
    Buzzer_Sing(SingCaliStart);
    imu->Calibrate();
    i = 0;
    while (!imu->DataReady() || !imu->CaliDone()) {
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
    float pitch_curr, yaw_curr;
    pitch_curr = -imu->INS_angle[2];
    yaw_curr = imu->INS_angle[0];
    float pitch_target = 0, yaw_target = 0;

    float actural_chassis_turn_speed = chassis_vt / 6.0f;
    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
            continue;
        }
        if (!pitch_motor->IsEnable())
            pitch_motor->Enable();
        if (!yaw_motor->IsEnable())
            yaw_motor->Enable();
        pitch_curr = -imu->INS_angle[2];
        yaw_curr = imu->INS_angle[0];
        //        pitch_curr = witimu->INS_angle[0];
        //        yaw_curr = wrap<float>(witimu->INS_angle[2]-yaw_offset, -PI, PI);
        //    if (dbus->swr == remote::UP) {
        //      gimbal->TargetAbs(0, 0);
        //      gimbal->Update();
        //      pitch_target = pitch_curr;
        //      yaw_target = yaw_curr;
        //      control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        //      osDelay(1);
        //      continue;
        //    }
        if (dbus->IsOnline()) {
            if (dbus->mouse.y != 0) {
                pitch_ratio = dbus->mouse.y / 32767.0 * 7.5 / 7.0;
            } else {
                pitch_ratio = -dbus->ch3 / 18000.0 / 7.0;
            }
            if (dbus->mouse.x != 0) {
                yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
            } else {
                yaw_ratio = -dbus->ch2 / 18000.0 / 7.0;
            }
        } else if (refereerc->IsOnline()) {
            pitch_ratio = refereerc->remote_control.mouse.y / 32767.0 * 7.5 / 7.0;
            yaw_ratio = -refereerc->remote_control.mouse.x / 32767.0 * 7.5 / 7.0;
        } else {
            pitch_ratio = 0;
            yaw_ratio = 0;
        }

        // 根据遥控器输入计算目标角度，并且进行限幅
        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = wrap<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        //        if (-0.005 < pitch_diff && pitch_diff < 0.005) {
        //            pitch_diff = 0;
        //        }

        //TODO 等待标定
        const float offset_ratio =
            0.185;  // 底盘给出速度：31.416rad/s，实际速度：20*2*PI/21=5.81rad/s，计算可得比率大约为0.185
        const float offset_filter_ratio =
            0.02;  // 由于底盘相应延迟所以需要有延迟滤波，在跟随模式和小陀螺模式下切换，观察云台在启停时是否偏向一侧
        static float speed_offset = 0;
        speed_offset = (chassis_vt * offset_ratio) * offset_filter_ratio +
                       speed_offset * (1 - offset_filter_ratio);
        yaw_motor->SetSpeedOffset(speed_offset);

        float pitch_speed_offset = pitch_ratio;
        pitch_motor->SetSpeedOffset(pitch_speed_offset);
        switch (remote_mode) {
            case REMOTE_MODE_SPIN:
            case REMOTE_MODE_FOLLOW:
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->UpdateIMU(pitch_curr, yaw_curr);
                break;
            case REMOTE_MODE_ADVANCED:
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->Update();
                break;
            case REMOTE_MODE_AUTOPILOT:
                if (static_cast<uint8_t>(minipc->target_angle.accuracy) < 60 ||
                    abs(minipc->target_angle.target_pitch) > 90.0f ||
                    abs(minipc->target_angle.target_yaw) > 180.0f
                    )
                    break;
                gimbal->TargetAbs(minipc->target_angle.target_pitch,
                                  -minipc->target_angle.target_yaw);
                gimbal->UpdateIMU(pitch_curr, yaw_curr);
                break;
            default:
                kill_gimbal();
        }

        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal() {
    pitch_motor = new driver::Motor6020(can1, 0x205, 0x1FF);
    yaw_motor = new driver::Motor6020(can1, 0x206, 0x1FF);

    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 12,
        .ki = 0,
        .kd = 10,
        .max_out = 6 * PI,  // 最高旋转速度
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 8192,
        .ki = 0,
        .kd = 0,
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
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    pitch_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                         driver::MotorCANBase::ABSOLUTE);

    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_theta_pid_init = {
        .kp = 13,
        .ki = 0,
        .kd = 4.5, //4.5
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_omega_pid_init = {
        .kp = 4000, //4000
        .ki = 0,
        .kd = 2000, //2000
        .max_out = 16383,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 1.5 * PI,                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.5,    // 输出滤波系数
        .derivative_filtering_coefficient = 0.0003,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    yaw_motor->ReInitPID(yaw_omega_pid_init, driver::MotorCANBase::OMEGA);
    yaw_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA |
                       driver::MotorCANBase::ABSOLUTE);
    yaw_motor->SetSpeedFilter(0.03);

    // 初始化云台对象
    control::gimbal_t gimbal_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}
void kill_gimbal() {
    yaw_motor->Disable();
    pitch_motor->Disable();
    // steering_motor->SetOutput(0);
}