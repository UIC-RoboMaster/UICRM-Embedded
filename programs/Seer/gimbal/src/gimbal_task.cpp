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

#include "PWMMotorBase.h"
#include "chassis_task.h"
#ifdef HAS_MINIPC
#include "minipc_task.h"
#endif

osThreadId_t gimbalTaskHandle;

driver::Motor6020* pitch_motor = nullptr;
driver::Motor6020* upper_yaw_motor = nullptr;
driver::Motor6020* lower_yaw_motor = nullptr;


control::Dual_Yaw_Gimbal* gimbal = nullptr;
control::dual_yaw_gimbal_data_t* gimbal_param = nullptr;
driver::PWMMotorBase* bulletCap = nullptr;
static float yaw_speed_offset = 0;
float pitch_diff, yaw_diff;
void gimbalTask(void* arg) {
    UNUSED(arg);

    pitch_motor->Disable();
    upper_yaw_motor->Disable();
    lower_yaw_motor->Disable();

    osDelay(1500);
    while (!is_activate) {
        kill_gimbal();
        osDelay(GIMBAL_OS_DELAY);
    }
    int i = 0;
    while (i < 5000 || !imu->DataReady()) {
        while (!is_activate) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
        }
        if (!pitch_motor->IsOnline() || !upper_yaw_motor->IsOnline() || !lower_yaw_motor->IsOnline()) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
            continue;
        }
        if (!pitch_motor->IsEnable())
            pitch_motor->Enable();
        if (!upper_yaw_motor->IsEnable())
            upper_yaw_motor->Enable();
        if (!lower_yaw_motor->IsEnable())
            lower_yaw_motor->Enable();

        gimbal->TargetAbs(0, 0);
        gimbal->UpdateEncoder();

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
        if (!is_activate || !pitch_motor->IsOnline() || !upper_yaw_motor->IsOnline() ||
            !lower_yaw_motor->IsOnline()) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
            continue;
        }
        pitch_motor->Enable();
        upper_yaw_motor->Enable();
        lower_yaw_motor->Enable();
        gimbal->TargetAbs(0, 0);
        gimbal->UpdateEncoder();

        osDelay(1);
        ++i;
    }
    // 校准完成后播放一段音乐
    Buzzer_Sing(SingCaliDone);
    osDelay(100);

    // 初始化当前陀螺仪角度、遥控器输入转换的角度、目标角度
    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    float pitch_target = 0, yaw_target = 0;

    //    float actural_chassis_turn_speed = chassis_vt / 6.0f;
    while (true) {
        bool gimbal_en = is_activate && pitch_motor->IsOnline() && upper_yaw_motor->IsOnline() &&
                         lower_yaw_motor->IsOnline();
#ifdef HAS_REFEREE
        gimbal_en = gimbal_en && referee->game_robot_status.mains_power_gimbal_output;
#endif
        if (!gimbal_en) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
            continue;
        }

        // 子弹盖
        if (!bulletCap->isEnable())
            bulletCap->Enable();
        switch (bullet_cap_mode) {
            case BULLET_CAP_MODE_CLOSE:
                bulletCap->SetOutput(200);
                break;
            case BULLET_CAP_MODE_OPEN:
                bulletCap->SetOutput(0);
                break;
            default:
                break;
        }

        if (!pitch_motor->IsEnable())
            pitch_motor->Enable();
        // pitch_motor->Disable();
        if (!upper_yaw_motor->IsEnable())
            upper_yaw_motor->Enable();
        if (!lower_yaw_motor->IsEnable())
            lower_yaw_motor->Enable();
        pitch_curr = -imu->INS_angle[1];
        yaw_curr = imu->INS_angle[0];
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
        } else if (refereerc != nullptr && refereerc->IsOnline()) {
            pitch_ratio = refereerc->remote_control.mouse.y / 32767.0 * 7.5 / 7.0;
            yaw_ratio = -refereerc->remote_control.mouse.x / 32767.0 * 7.5 / 7.0;
        } else {
            pitch_ratio = 0;
            yaw_ratio = 0;
        }

        // 根据遥控器输入计算目标角度，并且进行限幅
        pitch_target = clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        // 输入是朝向增量，小 yaw 的 ±90° 是关节限位，由双 yaw 组件协调。
        yaw_target = wrap<float>(yaw_ratio, -PI, PI);

        pitch_diff = clip<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        // pitch_diff *= -1;
        // yaw_diff *= -1;

        // if (-0.005 < pitch_diff && pitch_diff < 0.005) {
        //     pitch_diff = 0;
        // }

        // TODO 等待标定
        const float offset_ratio =
            0.185;  // 底盘给出速度：31.416rad/s，实际速度：20*2*PI/21=5.81rad/s，计算可得比率大约为0.185
        const float offset_filter_ratio =
            0.02;  // 由于底盘相应延迟所以需要有延迟滤波，在跟随模式和小陀螺模式下切换，观察云台在启停时是否偏向一侧
        yaw_speed_offset = (chassis_vt * offset_ratio) * offset_filter_ratio +
                           yaw_speed_offset * (1 - offset_filter_ratio);
        lower_yaw_motor->SetSpeedFeedforward(
            gimbal_param->lower_yaw_joint_inverted ? -yaw_speed_offset : yaw_speed_offset
        );

        // float pitch_speed_offset = pitch_ratio;
        // pitch_motor->SetSpeedOffset(pitch_speed_offset);

        switch (remote_mode) {
            case REMOTE_MODE_SPIN:
            case REMOTE_MODE_FOLLOW:
                gimbal->TargetRel(-pitch_diff, yaw_diff);
                break;
            case REMOTE_MODE_ADVANCED:
                gimbal->TargetRel(-pitch_diff, yaw_diff);
                break;
            case REMOTE_MODE_AUTOPILOT:
#ifdef HAS_MINIPC
                // if (!minipc->target_angle.shoot_cmd) {
                //     gimbal->TargetRel(0, 0.01 * PI / 180);
                //     break;
                // }
                if (  // static_cast<uint8_t>(minipc->target_angle.accuracy) < 60 || //accuracy not
                      // implemented yet
                    abs(minipc->target_angle.target_pitch) > (90.0f * PI / 180) ||
                    abs(minipc->target_angle.target_yaw) > (180.0f * PI / 180))
                    break;
                gimbal->TargetAbs(-minipc->target_angle.target_pitch, minipc->target_angle.target_yaw);
#else
                gimbal->TargetRel(-pitch_diff, yaw_diff);
#endif
                break;
            default:
                kill_gimbal();
                osDelay(GIMBAL_OS_DELAY);
                continue;
        }

        gimbal->UpdateIMU(-pitch_curr, yaw_curr);

        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal() {
    pitch_motor = new driver::Motor6020(can2, pitch_rx_id);
    upper_yaw_motor = new driver::Motor6020(can2, upper_yaw_rx_id);
    lower_yaw_motor = new driver::Motor6020(can1, lower_yaw_rx_id);

    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 60,
        .ki = 0.005,
        .kd = 4000,
        .max_out = 6 * PI,  // 最高旋转速度
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::DjiMotorBase::THETA);
    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 1500,
        .ki = 0,
        .kd = 500,
        .max_out = 16384,  // 最大电流输出，参考说明书
        .max_iout = 4000,
        .deadband = 0,                                                // 死区
        .A = 1.5 * PI,                                                // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                                  // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                          // 输出滤波系数
        .derivative_filtering_coefficient = 0,                        // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::DjiMotorBase::OMEGA);
    pitch_motor->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE);

    upper_yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t upper_yaw_theta_pid_init = {
        .kp = 10,
        .ki = 0,
        .kd = 0,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    upper_yaw_motor->ReInitPID(upper_yaw_theta_pid_init, driver::DjiMotorBase::THETA);
    control::ConstrainedPID::PID_Init_t yaw_omega_pid_init = {
        .kp = 80,
        .ki = 5,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                                          // 死区
        .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,             // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    upper_yaw_motor->ReInitPID(yaw_omega_pid_init, driver::DjiMotorBase::OMEGA);
    upper_yaw_motor->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE);


    // 大 yaw 单独使用慢轴 PID；初始值参考双 yaw 示例，现场按负载调参。
    lower_yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t lower_omega_pid_init = {
        .kp = 600,
        .ki = 30,
        .kd = 4,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,
        .A = 1.5 * PI,
        .B = 1 * PI,
        .output_filtering_coefficient = 1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
    lower_yaw_motor->ReInitPID(lower_omega_pid_init, driver::DjiMotorBase::OMEGA);
    control::ConstrainedPID::PID_Init_t lower_theta_pid_init = {
        .kp = 8.0f,
        .ki = 0.0f,
        .kd = 600.0f,
        .max_out = 6 * PI,
        .max_iout = 2000,
        .deadband = 0.0f,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 1,
        .derivative_filtering_coefficient = 0.0f,
        .mode = control::ConstrainedPID::Integral_Limit
    };
    lower_yaw_motor->ReInitPID(lower_theta_pid_init, driver::DjiMotorBase::THETA);
    lower_yaw_motor->SetMode(driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE);

    // 初始化云台对象
    control::dual_yaw_gimbal_t gimbal_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.upper_yaw_motor = upper_yaw_motor;
    gimbal_data.lower_yaw_motor = lower_yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal = new control::Dual_Yaw_Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();

    // init cap
    bulletCap = new driver::PWMMotorBase(&htim1, 1, 1000000, 50, 1440);  // 1435
}
void kill_gimbal() {
    yaw_speed_offset = 0;
    upper_yaw_motor->SetSpeedFeedforward(0);
    lower_yaw_motor->SetSpeedFeedforward(0);
    upper_yaw_motor->Disable();
    lower_yaw_motor->Disable();
    pitch_motor->Disable();
    bulletCap->Disable();
    // steering_motor->SetOutput(0);
}
