#pragma once
#include "controller.h"
#include "gimbal.h"

// basic information of gimbal
const control::gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 4.72515f,
    .yaw_offset_ = 3.6478f,
    .pitch_max_ = 0.4253f,
    .yaw_max_ = PI,
};
// basic information of gimbal
extern control::gimbal_pid_t gimbalBasicPID;
void init_gimbalBasicData();

// Spin PID of gimbal
extern control::gimbal_pid_t gimbalSpinPID;
void init_gimbalSpinData();