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
bool chassis_boost_flag = false;
const float speed_offset = 660;
const float speed_offset_boost = 1320;
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
    float spin_speed = 350;
    float manual_mode_yaw_pid_args[3] = {300, 0, 0};
    float manual_mode_yaw_pid_max_iout = 0;
    float manual_mode_yaw_pid_max_out = 350;
    control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
        manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
    manual_mode_pid->Reset();
    float manual_mode_pid_output = 0;
    float current_speed_offset = speed_offset;
    remote::keyboard_t keyboard;
    remote::keyboard_t last_keyboard;
    RampSource* vx_ramp = new RampSource(0, -chassis_vx_max / 2, chassis_vx_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    RampSource* vy_ramp = new RampSource(0, -chassis_vy_max / 2, chassis_vy_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    RampSource* vz_ramp = new RampSource(0, -chassis_vz_max / 2, chassis_vz_max / 2,
                                         1.0f / (CHASSIS_OS_DELAY * 1000));
    BoolEdgeDetector* w_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* s_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* a_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* d_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shift_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* q_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* e_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* x_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch0_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch1_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch2_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch3_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* ch4_edge = new BoolEdgeDetector(false);

    int random_spin_start_time = 0;
    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_chassis();
            osDelay(CHASSIS_OS_DELAY);

            continue;
        }
        {
            last_keyboard = keyboard;
            keyboard = dbus->keyboard;
        }
        {
            w_edge->input(keyboard.bit.W);
            s_edge->input(keyboard.bit.S);
            a_edge->input(keyboard.bit.A);
            d_edge->input(keyboard.bit.D);
            shift_edge->input(keyboard.bit.SHIFT);
            q_edge->input(keyboard.bit.Q);
            e_edge->input(keyboard.bit.E);
            x_edge->input(keyboard.bit.X);
            ch0_edge->input(dbus->ch0 != 0);
            ch1_edge->input(dbus->ch1 != 0);
            ch2_edge->input(dbus->ch2 != 0);
            ch3_edge->input(dbus->ch3 != 0);
            ch4_edge->input(dbus->ch4 != 0);
        }

        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);
        if (shift_edge->posEdge()) {
            if (chassis_boost_flag) {
                chassis_boost_flag = false;
                vx_ramp->SetMax(chassis_vx_max / 2);
                vy_ramp->SetMax(chassis_vy_max / 2);
                vz_ramp->SetMax(chassis_vz_max / 2);
                vx_ramp->SetMin(-chassis_vx_max / 2);
                vy_ramp->SetMin(-chassis_vy_max / 2);
                vz_ramp->SetMin(-chassis_vz_max / 2);
                current_speed_offset = speed_offset;
            } else {
                chassis_boost_flag = true;
                vx_ramp->SetMax(chassis_vx_max);
                vy_ramp->SetMax(chassis_vy_max);
                vz_ramp->SetMax(chassis_vz_max);
                vx_ramp->SetMin(-chassis_vx_max);
                vy_ramp->SetMin(-chassis_vy_max);
                vz_ramp->SetMin(-chassis_vz_max);
                current_speed_offset = speed_offset_boost;
            }
        }
        if (ch0_edge->get()) {
            vx_set_org = dbus->ch0;
        } else if (ch0_edge->negEdge() || x_edge->posEdge()) {
            vx_set_org = 0;
            vx_ramp->SetCurrent(0);
        } else if (d_edge->get()) {
            vx_set_org = vx_ramp->Calc(current_speed_offset);
        } else if (a_edge->get()) {
            vx_set_org = vx_ramp->Calc(-current_speed_offset);
        } else {
            if (vx_set_org > 0) {
                vx_set_org = vx_ramp->Calc(-current_speed_offset);
            } else if (vx_set_org < 0) {
                vx_set_org = vx_ramp->Calc(current_speed_offset);
            }
        }
        if (ch1_edge->get()) {
            vy_set_org = dbus->ch1;
        } else if (ch1_edge->negEdge() || x_edge->posEdge()) {
            vy_set_org = 0;
            vy_ramp->SetCurrent(0);
        } else if (w_edge->get()) {
            vy_set_org = vy_ramp->Calc(current_speed_offset);
        } else if (s_edge->get()) {
            vy_set_org = vy_ramp->Calc(-current_speed_offset);
        } else {
            if (vy_set_org > 0) {
                vy_set_org = vy_ramp->Calc(-current_speed_offset);
            } else if (vy_set_org < 0) {
                vy_set_org = vy_ramp->Calc(current_speed_offset);
            }
        }
        if (ch4_edge->get()) {
            offset_yaw = dbus->ch4;
        } else if (ch4_edge->negEdge() || x_edge->posEdge()) {
            offset_yaw = 0;
            vz_ramp->SetCurrent(0);
        } else if (e_edge->get()) {
            offset_yaw = vz_ramp->Calc(current_speed_offset);
        } else if (q_edge->get()) {
            offset_yaw = vz_ramp->Calc(-current_speed_offset);
        } else {
            if (offset_yaw > 0) {
                offset_yaw = vz_ramp->Calc(-current_speed_offset);
            } else if (offset_yaw < 0) {
                offset_yaw = vz_ramp->Calc(current_speed_offset);
            }
        }
        chassis_vx = vx_set_org;
        chassis_vy = vy_set_org;
        chassis_vz = offset_yaw;
        vx_set = cos_yaw * vx_set_org + sin_yaw * vy_set_org;
        vy_set = -sin_yaw * vx_set_org + cos_yaw * vy_set_org;
        switch (remote_mode) {
            case REMOTE_MODE_FOLLOW:
                manual_mode_pid_output = manual_mode_pid->ComputeOutput(
                    yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_));
                chassis->SetSpeed(vx_set, vy_set, manual_mode_pid_output);
                chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                                referee->power_heat_data.chassis_power,
                                (float)referee->power_heat_data.chassis_power_buffer);
                break;
            case REMOTE_MODE_RANDOMSPIN:
                if(HAL_GetTick() - random_spin_start_time > 3000){
                    random_spin_start_time = HAL_GetTick();
                    float random_spin_speed = (float)(rand() % 198)+463;
                    if(spin_speed > 0) {
                        spin_speed = random_spin_speed;
                    } else {
                        spin_speed = -random_spin_speed;
                    }
                }
                vz_set = spin_speed;
                chassis->SetSpeed(vx_set, vy_set, vz_set);
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
    chassis = new control::Chassis(chassis_data, 0.04);
}
void kill_chassis() {
    control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
    fl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    br_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors, 4);
}