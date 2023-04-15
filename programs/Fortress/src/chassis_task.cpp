#include "chassis_task.h"
osThreadId_t chassisTaskHandle;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vz = 0;
void chassisTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);
    control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

    while (remote_mode == REMOTE_MODE_KILL) {
        kill_chassis();
        osDelay(CHASSIS_OS_DELAY);
    }

    while (!imu->DataReady() || !imu->CaliDone()) {
        osDelay(1);
    }

    float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    // float last_speed = 0;
    float sin_yaw, cos_yaw, vx_set = 0, vy_set = 0, vz_set = 0, vx_set_org = 0, vy_set_org = 0;
    float offset_yaw = 0;
    float spin_speed = 500;
    float manual_mode_yaw_pid_args[3] = {500, 0, 0};
    float manual_mode_yaw_pid_max_iout = 0;
    float manual_mode_yaw_pid_max_out = 660;
    control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
        manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
    manual_mode_pid->Reset();
    float manual_mode_pid_output = 0;
    const float keyboard_step = 60;
    remote::keyboard_t keyboard;
    remote::keyboard_t last_keyboard;
    RemoteMode current_mode = remote_mode;
    RemoteMode lastmode;
    int16_t ch0 = 0;
    int16_t ch1 = 0;
    //    int16_t ch2 = 0;
    //    int16_t ch3 = 0;
    int16_t ch4 = 0;
    int16_t last_ch0 = 0;
    int16_t last_ch1 = 0;
    //    int16_t last_ch2 = 0;
    //    int16_t last_ch3 = 0;
    int16_t last_ch4 = 0;
    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            osDelay(CHASSIS_OS_DELAY);

            continue;
        }
        last_keyboard = keyboard;
        keyboard = dbus->keyboard;
        lastmode = current_mode;
        current_mode = remote_mode;
        last_ch0 = ch0;
        last_ch1 = ch1;
        //        last_ch2 = ch2;
        //        last_ch3 = ch3;
        last_ch4 = ch4;
        ch0 = dbus->ch0;
        ch1 = dbus->ch1;
        //        ch2 = dbus->ch2;
        //        ch3 = dbus->ch3;
        ch4 = dbus->ch4;
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);
        if (ch0 != 0) {
            vx_set_org = ch0;
        } else if ((keyboard.bit.X == 1 && last_keyboard.bit.X == 0) ||
                   (last_ch0 != 0 && ch0 == 0) || (current_mode != lastmode)) {
            vx_set_org = 0;
        } else if (keyboard.bit.A == 1 && last_keyboard.bit.A == 0) {
            vx_set_org -= keyboard_step;
        } else if (keyboard.bit.D == 1 && last_keyboard.bit.D == 0) {
            vx_set_org += keyboard_step;
        }
        vx_set_org = clip<float>(vx_set_org, -660, 660);
        if (ch1 != 0) {
            vy_set_org = ch1;
        } else if ((keyboard.bit.X == 1 && last_keyboard.bit.X == 0) ||
                   (last_ch1 != 0 && ch1 == 0) || (current_mode != lastmode)) {
            vy_set_org = 0;
        } else if (keyboard.bit.W == 1 && last_keyboard.bit.W == 0) {
            vy_set_org += keyboard_step;
        } else if (keyboard.bit.S == 1 && last_keyboard.bit.S == 0) {
            vy_set_org -= keyboard_step;
        }
        vy_set_org = clip<float>(vy_set_org, -660, 660);
        if (ch4 != 0) {
            offset_yaw = ch4;
        } else if ((keyboard.bit.X == 1 && last_keyboard.bit.X == 0) ||
                   (last_ch4 != 0 && ch4 == 0) || (current_mode != lastmode)) {
            offset_yaw = 0;
        } else if (keyboard.bit.Q == 1 && last_keyboard.bit.Q == 0) {
            offset_yaw += keyboard_step;
        } else if (keyboard.bit.E == 1 && last_keyboard.bit.E == 0) {
            offset_yaw -= keyboard_step;
        }
        offset_yaw = clip<float>(offset_yaw, -660, 660);
        chassis_vx = vx_set_org;
        chassis_vy = vy_set_org;
        vx_set = cos_yaw * vx_set_org + sin_yaw * vy_set_org;
        vy_set = -sin_yaw * vx_set_org + cos_yaw * vy_set_org;
        switch (remote_mode) {
            case REMOTE_MODE_MANUAL:
                manual_mode_pid_output = manual_mode_pid->ComputeOutput(
                    yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_));
                chassis->SetSpeed(vx_set, vy_set, manual_mode_pid_output);
                chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                                referee->power_heat_data.chassis_power,
                                (float)referee->power_heat_data.chassis_power_buffer);
                break;
            case REMOTE_MODE_SPIN:

                if (offset_yaw != 0) {
                    spin_speed = spin_speed + offset_yaw;
                    offset_yaw = 0;
                    spin_speed = clip<float>(spin_speed, -660, 660);
                }
                vz_set = spin_speed;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
                chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                                referee->power_heat_data.chassis_power,
                                (float)referee->power_heat_data.chassis_power_buffer);
                break;
            case REMOTE_MODE_ADVANCED:
                vz_set = offset_yaw;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
                chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                                referee->power_heat_data.chassis_power,
                                (float)referee->power_heat_data.chassis_power_buffer);
                break;
            default:
                // Not Support
                kill_chassis();
        }
        chassis_vz = vz_set;
        control::MotorCANBase::TransmitOutput(motors, 4);
        osDelay(CHASSIS_OS_DELAY);
    }
}

void init_chassis() {
    fl_motor = new control::Motor3508(can1, 0x202);
    fr_motor = new control::Motor3508(can1, 0x201);
    bl_motor = new control::Motor3508(can1, 0x203);
    br_motor = new control::Motor3508(can1, 0x204);

    control::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);
}
void kill_chassis() {
    control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
    fl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    br_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors, 4);
}