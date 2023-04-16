//
// Created by yangr on 2023/4/15.
//

#include "gimbal_data.h"

control::gimbal_pid_t gimbalBasicPID;

void init_gimbalBasicData() {
    float pitch_theta_max_iout = 0;
    float pitch_theta_max_out = 10;
    float pitch_omega_max_iout = 10000;
    float pitch_omega_max_out = 30000;
    float yaw_theta_max_iout = 0;
    float yaw_theta_max_out = 10;
    float yaw_omega_max_iout = 5000;  // 10000
    float yaw_omega_max_out = 30000;
    float* pitch_theta_pid_param = new float[3]{15, 0, 0};
    float* pitch_omega_pid_param = new float[3]{2900, 60, 0};
    float* yaw_theta_pid_param = new float[3]{26, 0, 0.3};
    float* yaw_omega_pid_param = new float[3]{3600, 20, 0};
    gimbalBasicPID.pitch_theta_pid = new control::ConstrainedPID(
        pitch_theta_pid_param, pitch_theta_max_iout, pitch_theta_max_out);
    gimbalBasicPID.pitch_omega_pid = new control::ConstrainedPID(
        pitch_omega_pid_param, pitch_omega_max_iout, pitch_omega_max_out);
    gimbalBasicPID.yaw_theta_pid =
        new control::ConstrainedPID(yaw_theta_pid_param, yaw_theta_max_iout, yaw_theta_max_out);
    gimbalBasicPID.yaw_omega_pid =
        new control::ConstrainedPID(yaw_omega_pid_param, yaw_omega_max_iout, yaw_omega_max_out);
}

control::gimbal_pid_t gimbalSpinPID;
void init_gimbalSpinData() {
    float pitch_theta_max_iout = 0;
    float pitch_theta_max_out = 10;
    float pitch_omega_max_iout = 10000;
    float pitch_omega_max_out = 30000;
    float yaw_theta_max_iout = 0;
    float yaw_theta_max_out = 10;
    float yaw_omega_max_iout = 5000;  // 10000
    float yaw_omega_max_out = 30000;
    float* pitch_theta_pid_param = new float[3]{15, 0, 0};
    float* pitch_omega_pid_param = new float[3]{2900, 60, 0};
    float* yaw_theta_pid_param = new float[3]{26, 0, 0.3};
    float* yaw_omega_pid_param = new float[3]{3600, 20, 0};
    gimbalBasicPID.pitch_theta_pid = new control::ConstrainedPID(
        pitch_theta_pid_param, pitch_theta_max_iout, pitch_theta_max_out);
    gimbalBasicPID.pitch_omega_pid = new control::ConstrainedPID(
        pitch_omega_pid_param, pitch_omega_max_iout, pitch_omega_max_out);
    gimbalBasicPID.yaw_theta_pid =
        new control::ConstrainedPID(yaw_theta_pid_param, yaw_theta_max_iout, yaw_theta_max_out);
    gimbalBasicPID.yaw_omega_pid =
        new control::ConstrainedPID(yaw_omega_pid_param, yaw_omega_max_iout, yaw_omega_max_out);
}