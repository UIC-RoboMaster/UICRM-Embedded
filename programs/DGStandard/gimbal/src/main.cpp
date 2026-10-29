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
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "public_port.h"
#include "printf.h"
#include "remote_task.h"
#include "shoot_task.h"

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
}

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);
    // 调试信息走 UART6（921600），配合 USB-TTL 下载器在电脑上查看。
    print_use_uart(&huart6, true, 921600);

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
    // 板级默认任务已初始化 USB，给电脑留出枚举设备的时间。
    osDelay(1000);

    while (true) {
        // 先输出不含浮点数的版本标记，即使 KILL 状态也能确认烧录与 USB 打印。
        const uint32_t now = bsp::GetHighresTickMilliSec();
        diagnostic_line("DGSTD RESP-V4 t=%lu phase=%s mode=%s DBUS=%d SWR=%d CH2=%d CH3=%d\r\n",
                        (unsigned long)now, gimbal_phase, remote_mode_str(remote_mode),
                        (int)dbus->IsOnline(), (int)dbus->swr, (int)dbus->ch2, (int)dbus->ch3);
        const uint32_t last_rx = dbus->GetLastUptime();
        diagnostic_line("RC ok=%lu bad=%lu age_ms=%lu | inhibit=%lu last=%lu stops=%lu\r\n",
                        (unsigned long)dbus->accepted_frames, (unsigned long)dbus->rejected_frames,
                        (unsigned long)(last_rx == 0 ? 0 : bsp::GetHighresTickMilliSec() - last_rx),
                        (unsigned long)gimbal_inhibit_mask, (unsigned long)gimbal_last_inhibit_mask,
                        (unsigned long)gimbal_disable_events);
        const auto pitch_theta = pitch_motor->GetPIDState(driver::DjiMotorBase::THETA);
        const auto pitch_omega = pitch_motor->GetPIDState(driver::DjiMotorBase::OMEGA);
        const auto yaw_theta = yaw_motor->GetPIDState(driver::DjiMotorBase::THETA);
        const auto yaw_omega = yaw_motor->GetPIDState(driver::DjiMotorBase::OMEGA);
        // 每 200ms 输出一帧；USB 驱动复制到独立发送缓冲区。
        // err 是角度差，vcmd 是角度环要求的速度，i 是速度环积分输出。
        diagnostic_line("SWL=%d FRIC=%s LOAD=%s\r\n", (int)dbus->swl,
                        shoot_fric_mode_str(shoot_flywheel_mode), shoot_load_mode_str(shoot_load_mode));
        diagnostic_line("CH0=%d CH1=%d CH4=%d CHASSIS frame=%s tx_en=%d vx=%.3f vy=%.3f vt=%.3f\r\n",
                        (int)dbus->ch0, (int)dbus->ch1, (int)dbus->ch4,
                        chassis_gimbal_relative ? "GIMBAL" : "BODY",
                        (int)chassis_control_enabled, chassis_vx, chassis_vy, chassis_vt);
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
        diagnostic_line("Y diag slewed=%.4f peak_w=%.4f peak_out=%d\r\n",
                        yaw_motor->GetSpeedCommand(), yaw_peak_speed, yaw_peak_output);
        diagnostic_line("S enc=%.4f tgt=%.4f en=%d online=%d out=%d curr=%d\r\n",
                        steering_motor->GetOutputShaftTheta(), steering_motor->GetTarget(),
                        (int)steering_motor->IsEnable(), (int)steering_motor->IsOnline(),
                        (int)steering_motor->GetOutput(), (int)steering_motor->GetCurr());
        diagnostic_line("IMU cal=%d warm=%d yaw=%.4f pitch=%.4f roll=%.4f\r\n\r\n",
                        (int)imu->CaliDone(), (int)imu->DataReady(),
                        imu->INS_angle[0], imu->INS_angle[1], imu->INS_angle[2]);
        osDelay(200);
    }
}
