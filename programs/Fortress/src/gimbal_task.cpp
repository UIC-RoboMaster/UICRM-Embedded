/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

osThreadId_t gimbalTaskHandle;

control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
void gimbalTask(void* arg) {
    UNUSED(arg);

    control::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor, steering_motor};
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

        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        osDelay(GIMBAL_OS_DELAY);
        ++i;
    }

    // buzzer->SingTone(bsp::BuzzerNote::La6M);
    Buzzer_Sing(SingCaliStart);
    imu->Calibrate();
    i = 0;
    while (!imu->DataReady() || !imu->CaliDone()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        osDelay(1);
        ++i;
    }
    Buzzer_Sing(SingCaliDone);
    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    pitch_curr = imu->INS_angle[2];
    yaw_curr = imu->INS_angle[0];
    float pitch_target = 0, yaw_target = 0;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            kill_gimbal();
            osDelay(GIMBAL_OS_DELAY);
            continue;
        }

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
        if (dbus->mouse.y != 0) {
            pitch_ratio = -dbus->mouse.y / 32767.0 * 7.5 / 7.0;
        } else {
            pitch_ratio = dbus->ch3 / 18000.0 / 7.0;
        }
        if (dbus->mouse.x != 0) {
            yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
        } else {
            yaw_ratio = -dbus->ch2 / 18000.0 / 7.0;
        }

        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = wrap<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        //        if (-0.005 < pitch_diff && pitch_diff < 0.005) {
        //            pitch_diff = 0;
        //        }

        switch (remote_mode) {
            case REMOTE_MODE_SPIN:
            case REMOTE_MODE_RANDOMSPIN:
            case REMOTE_MODE_FOLLOW:
            case REMOTE_MODE_ADVANCED:
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->UpdateIMU(pitch_curr, yaw_curr);
                break;
                //                gimbal->TargetRel(pitch_diff, yaw_diff);
                //                gimbal->Update();
                //                break;
            default:
                kill_gimbal();
        }

        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        osDelay(GIMBAL_OS_DELAY);
    }
}

void init_gimbal() {
    init_gimbalBasicData();
    init_gimbalSpinData();
    pitch_motor = new control::Motor6020(can2, 0x206);
    yaw_motor = new control::Motor6020(can2, 0x205);
    control::gimbal_t gimbal_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal_data.data = gimbal_init_data;
    gimbal_data.pid = gimbalBasicPID;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}
void kill_gimbal() {
    control::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor, steering_motor};
    yaw_motor->SetOutput(0);
    pitch_motor->SetOutput(0);
    // steering_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
}