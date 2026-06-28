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

const float chassis_max_xy_speed = 2 * PI * 10;
const float chassis_max_t_speed = 2 * PI * 5;

float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;
bool chassis_boost_flag = true;

driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

driver::SuperCap* super_cap = nullptr;
control::Chassis* chassis = nullptr;
// bsp::BatteryVol* battery_vol = nullptr;

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
        if (dbus->IsOnline()) {
            keyboard = dbus->keyboard;
        } else if (refereerc->IsOnline()) {
            keyboard = refereerc->remote_control.keyboard;
        }

        // 以云台为基准的（整车的）运动速度，范围为[-1, 1]
        float car_vx, car_vy, car_vt;
        if (dbus->ch0 || dbus->ch1 || dbus->ch2 || dbus->ch3 || dbus->ch4) {
            // 优先使用遥控器
            const float speed_scale = 0.5;
            car_vx = (float)dbus->ch0 / dbus->ROCKER_MAX * speed_scale;
            car_vy = (float)dbus->ch1 / dbus->ROCKER_MAX * speed_scale;
            car_vt = (float)dbus->ch4 / dbus->ROCKER_MAX * speed_scale;
        } else {
            // 使用键盘
            const float keyboard_speed = keyboard.bit.SHIFT ? 1 : 0.5;
            const float keyboard_spin_speed = 1;
            car_vx = (keyboard.bit.D - keyboard.bit.A) * keyboard_speed;
            car_vy = (keyboard.bit.W - keyboard.bit.S) * keyboard_speed;
            car_vt = (keyboard.bit.E - keyboard.bit.Q) * keyboard_spin_speed;
        }

        // 云台相对底盘的角度，通过云台和底盘连接的电机获取
        float A = yaw_motor->GetTheta() - gimbal_param->yaw_offset_;
        // 云台当前相对云台零点的角度，通过IMU获取
        float B = INS_Angle.yaw;
        // 云台目标相对云台零点的角度，直接读取gimbal class获取
        float C = gimbal->getYawTarget() - gimbal_param->yaw_offset_;
        float chassis_target_diff = C - B + A;
        chassis_target_diff = -chassis_target_diff;
        chassis_target_diff = pitch_diff = wrap<float>(chassis_target_diff, -PI, PI);

        // 底盘以底盘自己为基准的运动速度
        float sin_yaw = arm_sin_f32(chassis_target_diff);
        float cos_yaw = arm_cos_f32(chassis_target_diff);
        chassis_vx = cos_yaw * car_vx + sin_yaw * car_vy;
        chassis_vy = -sin_yaw * car_vx + cos_yaw * car_vy;
        chassis_vt = 0;

        if (remote_mode == REMOTE_MODE_ADVANCED || remote_mode == REMOTE_MODE_AUTOAIM) {
            // 手动模式下，遥控器直接控制底盘速度
            chassis_vx = car_vx;
            chassis_vy = car_vy;
            chassis_vt = car_vt;
        }

        if (remote_mode == REMOTE_MODE_FOLLOW) {
            // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
            const float angle_threshold = 0.02f;
            float chassis_vt_pid_error = chassis_target_diff;
            if (fabs(chassis_vt_pid_error) < angle_threshold) {
                chassis_vt_pid_error = 0;
            }

            static control::ConstrainedPID* chassis_vt_pid = new control::ConstrainedPID(2 / (2 * PI), 0, 0, 0.5, 1);
            float vt = chassis_vt_pid->ComputeOutput(chassis_vt_pid_error);
            if (chassis_vt_pid_error != 0)
                chassis_vt = vt;
        }

        if (remote_mode == REMOTE_MODE_SPIN) {
            // 小陀螺模式，拨盘用来控制底盘加速度
            static float spin_speed = 1;
            spin_speed = spin_speed + car_vt * 0.01;
            spin_speed = clip<float>(spin_speed, -1, 1);
            chassis_vt = spin_speed;
        }

        // 进行缩放
        chassis_vx *= chassis_max_xy_speed;
        chassis_vy *= chassis_max_xy_speed;
        chassis_vt *= chassis_max_t_speed;

        static const float move_ease_ratio = 0.9;
        static const float turn_ease_ratio = 0.9;
        static Ease chassis_ease_vx(0, move_ease_ratio);
        static Ease chassis_ease_vy(0, move_ease_ratio);
        static Ease chassis_ease_vt(0, turn_ease_ratio);
        chassis_vx = chassis_ease_vx.Calc(chassis_vx);
        chassis_vy = chassis_ease_vy.Calc(chassis_vy);
        chassis_vt = chassis_ease_vt.Calc(chassis_vt);

        chassis->SetSpeed(chassis_vx, chassis_vy, chassis_vt);

#ifdef HAS_REFEREE
        uint8_t buffer_percent = referee->power_heat_data.chassis_power_buffer * 100 / 60;
        uint8_t max_watt = referee->game_robot_status.chassis_power_limit;
#else
        uint8_t buffer_percent = 50;
        uint8_t max_watt = 100;
#endif
        // chassis->UpdatePowerVoltage(battery_vol->GetBatteryVol());
        chassis->UpdatePower(true, max_watt, 0, buffer_percent);
        chassis->Update();
        osDelay(CHASSIS_OS_DELAY);
    }
}

void init_chassis() {
    // 初始化底盘电机
    fl_motor = new driver::Motor3508(can1, 0x201);
    fr_motor = new driver::Motor3508(can1, 0x202);
    bl_motor = new driver::Motor3508(can1, 0x203);
    br_motor = new driver::Motor3508(can1, 0x204);

    // 底盘电机PID参数
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                                          // 死区
        .A = 3 * PI,                                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fl_motor->SetMode(driver::MotorCANBase::OMEGA);
    fl_motor->SetTransmissionRatio(14);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fr_motor->SetMode(driver::MotorCANBase::OMEGA);
    fr_motor->SetTransmissionRatio(14);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    bl_motor->SetMode(driver::MotorCANBase::OMEGA);
    bl_motor->SetTransmissionRatio(14);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    br_motor->SetMode(driver::MotorCANBase::OMEGA);
    br_motor->SetTransmissionRatio(14);

    // 初始化超级电容
    driver::supercap_init_t supercap_init = {
        .can = can1,
        .tx_id = 0x02e,
        .tx_settings_id = 0x02f,
        .rx_id = 0x030,
    };
    super_cap = new driver::SuperCap(supercap_init);
    super_cap->Disable();
    super_cap->TransmitSettings();
    super_cap->Enable();
    super_cap->TransmitSettings();
    super_cap->SetMaxVoltage(24.0f);
    super_cap->SetPowerTotal(100.0f);
    super_cap->SetMaxChargePower(150.0f);
    super_cap->SetMaxDischargePower(250.0f);
    super_cap->SetPerferBuffer(50.0f);

    // 初始化电池电压采样
    // battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);

    // 初始化底盘
    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis_data.has_super_capacitor = true;
    chassis_data.super_capacitor = super_cap;
    chassis = new control::Chassis(chassis_data);

    chassis->SetMaxMotorSpeed(2 * PI * 7);
    chassis->Disable();
}

void kill_chassis() {
    chassis->Disable();
}
