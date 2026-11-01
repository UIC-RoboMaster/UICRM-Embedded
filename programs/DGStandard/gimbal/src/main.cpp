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

#include "main.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "chassis_control.h"
#include "diagnostic_frame.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "public_port.h"
#include "printf.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "task.h"

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

static dgstandard::DiagnosticFrame diagnostic_usb_frame;
static bsp::VirtualUSB* diagnostic_usb_receiver = nullptr;
static uint8_t* diagnostic_usb_rx_data = nullptr;
static uint32_t diagnostic_usb_rx_length = 0;

// TypeC 的 USB 虚拟串口与 UART6 是两个独立接口；整帧镜像到 USB。
static bool diagnostic_usb_idle() {
    taskENTER_CRITICAL();
    const auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    const bool idle = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc && cdc->TxState == 0;
    taskEXIT_CRITICAL();
    return idle;
}

static void diagnostic_usb_flush() {
    if (diagnostic_usb_frame.Length() == 0)
        return;
    taskENTER_CRITICAL();
    const auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc && cdc->TxState == 0)
        CDC_Transmit_FS(diagnostic_usb_frame.Data(), diagnostic_usb_frame.Length());
    taskEXIT_CRITICAL();
}

// 诊断信息在 Debug/Release 中均保留；每行有独立、有界的格式化缓冲区。
static void diagnostic_line(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length <= 0)
        return;
    const uint32_t written = length >= static_cast<int>(sizeof(buffer)) ? sizeof(buffer) - 1 : length;
    dump(buffer, written);
    diagnostic_usb_frame.Append(buffer, written);
}

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    // 调试信息走 UART6（921600），配合 USB-TTL 下载器在电脑上查看。
    print_use_uart(&huart6, true, 921600);
    // 注册 USB 接收对象，让串口助手误点发送时也能安全消费数据；日志发送用独立静态帧。
    diagnostic_usb_receiver = new bsp::VirtualUSB();
    diagnostic_usb_receiver->SetupRx(64);
    diagnostic_usb_receiver->SetupRxData(&diagnostic_usb_rx_data, &diagnostic_usb_rx_length);

    // DBUS 控制云台与发射机构；未安装的裁判、图传、自瞄不参与。
    init_can();
    init_imu();
    init_buzzer();
    init_remote();
    init_shoot();
    init_gimbal();
    init_chassis();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    // 等待控制任务完成初始启动。
    osDelay(1000);

    while (true) {
        diagnostic_usb_frame.Begin(diagnostic_usb_idle());
        // 先输出不含浮点数的版本标记，KILL 状态下也保留诊断。
        const uint32_t now = bsp::GetHighresTickMilliSec();
        diagnostic_line("DGSTD YAW-V11 USB+UART6 t=%lu phase=%s mode=%s DBUS=%d SWR=%d CH2=%d CH3=%d\r\n",
                        (unsigned long)now, gimbal_phase, remote_mode_str(remote_mode),
                        (int)dbus->IsOnline(), (int)dbus->swr, (int)dbus->ch2, (int)dbus->ch3);
        const uint32_t last_rx = dbus->GetLastUptime();
        diagnostic_line("RC age_ms=%lu | inhibit=%lu last=%lu stops=%lu\r\n",
                        (unsigned long)(last_rx == 0 ? 0 : bsp::GetHighresTickMilliSec() - last_rx),
                        (unsigned long)gimbal_inhibit_mask, (unsigned long)gimbal_last_inhibit_mask,
                        (unsigned long)gimbal_disable_events);
        const auto pitch_theta = pitch_motor->GetPIDState(driver::DjiMotorBase::THETA);
        const auto pitch_omega = pitch_motor->GetPIDState(driver::DjiMotorBase::OMEGA);
        const auto yaw_theta = yaw_motor->GetPIDState(driver::DjiMotorBase::THETA);
        const auto yaw_omega = yaw_motor->GetPIDState(driver::DjiMotorBase::OMEGA);
        // 每 200ms 输出一帧，保留当前 UART6、921600 的打印接口。
        // err 是角度差，vcmd 是角度环要求的速度，i 是速度环积分输出。
        diagnostic_line("SWL=%d FRIC=%s LOAD=%s\r\n", (int)dbus->swl,
                        shoot_fric_mode_str(shoot_flywheel_mode), shoot_load_mode_str(shoot_load_mode));
        diagnostic_line("CH0=%d CH1=%d CH4=%d CHASSIS frame=%s tx_en=%d vx=%.3f vy=%.3f vt=%.3f ferr=%.4f\r\n",
                        (int)dbus->ch0, (int)dbus->ch1, (int)dbus->ch4,
                        chassis_gimbal_relative ? "GIMBAL" : "BODY",
                        (int)chassis_control_enabled, chassis_vx, chassis_vy, chassis_vt,
                        chassis_follow_error);
        diagnostic_line("TURN phase=%s spin_cmd=%.2f\r\n",
                        chassis_turn_phase, dgstandard::kSpinCommand);
        diagnostic_line("P enc=%.4f tgt=%.4f en=%d online=%d out=%d curr=%d w=%.4f\r\n",
                        pitch_motor->GetTheta(), pitch_motor->GetTarget(),
                        (int)pitch_motor->IsEnable(), (int)pitch_motor->IsOnline(),
                        (int)pitch_motor->GetOutput(), (int)pitch_motor->GetCurr(), pitch_motor->GetOmega());
        diagnostic_line("Y enc=%.4f tgt=%.4f en=%d online=%d out=%d curr=%d w=%.4f\r\n",
                        yaw_motor->GetTheta(), yaw_motor->GetTarget(),
                        (int)yaw_motor->IsEnable(), (int)yaw_motor->IsOnline(),
                        (int)yaw_motor->GetOutput(), (int)yaw_motor->GetCurr(), yaw_motor->GetOmega());
        diagnostic_line("P ctl err=%.4f vcmd=%.4f i=%.0f | Y ctl err=%.4f vcmd=%.4f i=%.0f\r\n",
                        pitch_theta.error, pitch_theta.output, pitch_omega.iout,
                        yaw_theta.error, yaw_theta.output, yaw_omega.iout);
        // 每次打印后重置统计，峰值与限幅次数只代表最近一个打印窗口。
        taskENTER_CRITICAL();
        const float peak_speed = yaw_peak_speed;
        const int peak_output = yaw_peak_output;
        const float peak_heading_error = yaw_peak_heading_error;
        const uint32_t samples = yaw_diagnostic_samples;
        const uint32_t output_limited = yaw_output_limit_samples;
        const uint32_t compensation_limited = yaw_compensation_limit_samples;
        yaw_peak_speed = yaw_peak_heading_error = 0;
        yaw_peak_output = 0;
        yaw_diagnostic_samples = yaw_output_limit_samples = yaw_compensation_limit_samples = 0;
        taskEXIT_CRITICAL();
        diagnostic_line("Y diag window_n=%lu peak_w=%.4f peak_out=%d peak_err=%.4f out_limit_n=%lu ff_limit_n=%lu\r\n",
                        (unsigned long)samples, peak_speed, peak_output, peak_heading_error,
                        (unsigned long)output_limited, (unsigned long)compensation_limited);
        diagnostic_line("Y world ready=%d sign=-1 tgt=%.4f actual=%.4f err=%.4f\r\n",
                        (int)yaw_world_ready, yaw_world_target, -INS_Angle.yaw, yaw_world_error);
        diagnostic_line("Y stab base_w=%.4f comp_w=%.4f heading_err=%.4f\r\n",
                        yaw_base_rate, yaw_compensation_rate,
                        dgstandard::WrapYawError(yaw_world_target + INS_Angle.yaw));
        diagnostic_line("S enc=%.4f tgt=%.4f en=%d online=%d out=%d curr=%d\r\n",
                        steering_motor->GetOutputShaftTheta(), steering_motor->GetTarget(),
                        (int)steering_motor->IsEnable(), (int)steering_motor->IsOnline(),
                        (int)steering_motor->GetOutput(), (int)steering_motor->GetCurr());
        diagnostic_line("FLY online=%d/%d out=%d/%d curr=%d/%d\r\n",
                        (int)flywheel_left->IsOnline(), (int)flywheel_right->IsOnline(),
                        (int)flywheel_left->GetOutput(), (int)flywheel_right->GetOutput(),
                        (int)flywheel_left->GetCurr(), (int)flywheel_right->GetCurr());
        diagnostic_line("FLY speed tgt=%.1f/%.1f w=%.1f/%.1f rad/s\r\n",
                        flywheel_left->GetTarget(), flywheel_right->GetTarget(),
                        flywheel_left->GetOutputShaftOmega(), flywheel_right->GetOutputShaftOmega());
        diagnostic_line("IMU cal=%d warm=%d yaw=%.4f pitch=%.4f roll=%.4f\r\n",
                        (int)imu->CaliDone(), (int)imu->DataReady(),
                        imu->INS_angle[0], imu->INS_angle[1], imu->INS_angle[2]);
        diagnostic_line("IMU temp=%.2f age_ms=%lu samples=%lu fresh=%d\r\n\r\n",
                        imu->Temp, (unsigned long)imu_attitude_age_ms(),
                        (unsigned long)imu_attitude_samples, (int)imu_attitude_fresh());
        diagnostic_usb_flush();
        osDelay(200);
    }
}
