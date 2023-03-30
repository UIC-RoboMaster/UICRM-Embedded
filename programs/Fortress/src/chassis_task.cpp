#include "chassis_task.h"
osThreadId_t chassisTaskHandle;
static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;

static control::Chassis* chassis = nullptr;
void chassisTask(void* arg) {
    UNUSED(arg);

    control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

    while (true) {
        //TODO: Wait until the system up
        osDelay(10);
    }
    while (!imu->DataReady() || !imu->CaliDone()) {
        osDelay(1);
    }

    float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    // float last_speed = 0;
    float sin_yaw, cos_yaw, vx_set, vy_set, vz_set, vx_set_org, vy_set_org;
    while (true) {
        //TODO: Offline Check
        //TODO: Mode Check
        if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) {
            while (true) {
                if (dbus->keyboard.bit.V || dbus->swr != remote::DOWN) {
                    break;
                }
                osDelay(10);
            }
            continue;
        }
        if (dbus->swr == remote::UP) {
            chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
        } else {
            relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

            sin_yaw = arm_sin_f32(relative_angle);
            cos_yaw = arm_cos_f32(relative_angle);
            vx_set_org = dbus->ch0;
            vy_set_org = dbus->ch1;
            vx_set = cos_yaw * vx_set_org + sin_yaw * vy_set_org;
            vy_set = -sin_yaw * vx_set_org + cos_yaw * vy_set_org;
            vz_set = dbus->ch2;
            chassis->SetSpeed(vx_set, vy_set, vz_set);
        }

        chassis->Update(false, 30, 20, 60);
        control::MotorCANBase::TransmitOutput(motors, 4);
        osDelay(CHASSIS_OS_DELAY);
    }
}

void init_chassis(){
    can1 = new bsp::CAN(&hcan1, 0x201, true);
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