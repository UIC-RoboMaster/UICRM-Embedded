#include "gimbal_task.h"


osThreadId_t gimbalTaskHandle;

bsp::CAN* can2 = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;

void gimbalTask(void* arg) {
    UNUSED(arg);

    control::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor, steering_motor};
    osDelay(2000);

    int i = 0;
    while (i < 5000 || !imu->DataReady()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        osDelay(1);
        ++i;
    }

    buzzer->SingTone(bsp::BuzzerNote::La6M);
    imu->Calibrate();

    i = 0;
    while (!imu->DataReady() || !imu->CaliDone()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        osDelay(1);
        ++i;
    }
    buzzer->SingTone(bsp::BuzzerNote::Silent);
    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    float pitch_target = 0, yaw_target = 0;
    float pitch_diff, yaw_diff;

    while (true) {

        pitch_curr = imu->INS_angle[2];
        yaw_curr = imu->INS_angle[0];
        //    if (dbus->swr == remote::UP) {
        //      gimbal->TargetAbs(0, 0);
        //      gimbal->Update();
        //      pitch_target = pitch_curr;
        //      yaw_target = yaw_curr;
        //      control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        //      osDelay(1);
        //      continue;
        //    }
        pitch_ratio = dbus->mouse.y / 32767.0 * 7.5 / 7.0;
        yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
        pitch_ratio = dbus->ch3 / 18000.0 / 7.0;
        yaw_ratio = dbus->ch4 / 18000.0 / 7.0;
        pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                                   gimbal_param->pitch_max_);
        yaw_target =
            wrap<float>(yaw_target + yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
        yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

        if (-0.005 < pitch_diff && pitch_diff < 0.005) {
            pitch_diff = 0;
        }
        if (dbus->swr == remote::UP) {
            gimbal->TargetAbsYawRelPitch(pitch_diff, 0);
            gimbal->Update();
            yaw_target = yaw_curr;
        } else {
            gimbal->TargetRel(pitch_diff, yaw_diff);
        }

        gimbal->Update();
        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal(){
    can2 = new bsp::CAN(&hcan2, 0x201, false);
    pitch_motor = new control::Motor6020(can2, 0x206);
    yaw_motor = new control::Motor6020(can2, 0x205);
    steering_motor = new control::Motor2006(can2, 0x207);
    control::gimbal_t gimbal_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.model = control::GIMBAL_FORTRESS;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}