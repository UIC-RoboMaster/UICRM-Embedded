#include "chassis_task.h"
osThreadId_t chassisTaskHandle;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;

float chassis_keyboard_mode_speed = 500;

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
    float sin_yaw, cos_yaw, vx_set, vy_set, vz_set, vx_set_org, vy_set_org;
    float offset_yaw;
    float spin_speed = 500;
    float manual_mode_yaw_pid_args[3] = {500, 0, 0};
    float manual_mode_yaw_pid_max_iout = 0;
    float manual_mode_yaw_pid_max_out = 660;

    control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
        manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
    manual_mode_pid->Reset();
    float manual_mode_pid_output = 0;
    RampSource* ramp_x = new RampSource(0,-660,660,0.001);
    RampSource* ramp_y = new RampSource(0,-660,660,0.001);
    float ramp_offset = 0;
    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            osDelay(CHASSIS_OS_DELAY);

            continue;
        }
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);
        if (dbus->mouse.z != 0 && dbus->keyboard.bit.SHIFT == 0) {
            chassis_keyboard_mode_speed += dbus->mouse.z / 10.0;
            chassis_keyboard_mode_speed = clip<float>(chassis_keyboard_mode_speed, 0, 660);
        }
        ramp_x->SetMax(chassis_keyboard_mode_speed);
        ramp_y->SetMax(chassis_keyboard_mode_speed);
        ramp_x->SetMin(-chassis_keyboard_mode_speed);
        ramp_y->SetMin(-chassis_keyboard_mode_speed);
        ramp_offset = chassis_keyboard_mode_speed / 0.5;
        if (dbus->keyboard.bit.A) {
            vx_set_org = ramp_x->Calc(-ramp_offset);
        } else if (dbus->keyboard.bit.D) {
            vx_set_org = ramp_x->Calc(ramp_offset);
        } else {
            ramp_x->SetCurrent(0);
            vx_set_org = dbus->ch0;
        }
        if (dbus->keyboard.bit.W) {
            vy_set_org = ramp_y->Calc(ramp_offset);
        } else if (dbus->keyboard.bit.S) {
            vy_set_org = ramp_y->Calc(-ramp_offset);
        } else {
            ramp_y->SetCurrent(0);
            vy_set_org = dbus->ch1;
        }
        if (dbus->keyboard.bit.Q) {
            offset_yaw = chassis_keyboard_mode_speed / 3;
        } else if (dbus->keyboard.bit.E) {
            offset_yaw = -chassis_keyboard_mode_speed / 3;
        } else {
            offset_yaw = dbus->ch4;
        }
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
                    spin_speed = spin_speed + (float)(dbus->ch4) / 200;
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