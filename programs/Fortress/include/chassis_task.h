#pragma once
#include "bsp_can.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "motor.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"
#include "utils.h"
extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
void chassisTask(void* arg);
void init_chassis();
void kill_chassis();
extern control::Chassis* chassis;
extern control::MotorCANBase* fl_motor;
extern control::MotorCANBase* fr_motor;
extern control::MotorCANBase* bl_motor;
extern control::MotorCANBase* br_motor;
extern float chassis_vx;
extern float chassis_vy;
extern float chassis_vz;
extern bool chassis_boost_flag;
const float chassis_vx_max = 660.0f;
const float chassis_vy_max = 660.0f;
const float chassis_vz_max = 660.0f;