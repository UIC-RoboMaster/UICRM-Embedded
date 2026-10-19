// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
// SPDX-License-Identifier: GPL-3.0-or-later

#include "main.h"

#include "bsp_can.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dm_imu.h"
#include "usart.h"

static bsp::CAN* can2 = nullptr;
static imu::DMIMU* dm_imu = nullptr;

/**
 * @brief 初始化 UART1 调试输出、CAN2 管理对象和 DM-IMU 驱动。
 * @note 调试串口为 115200、8N1；模块命令 ID 为 0x01，反馈 ID 为 0x11。
 */
void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 115200);
    can2 = new bsp::CAN(&hcan2, false);
    imu::DMIMU_init_t imu_init = {};
    imu_init.can = can2;
    imu_init.tx_id = 0x01;
    imu_init.rx_id = 0x11;
    dm_imu = new imu::DMIMU(imu_init);
}

/**
 * @brief 本例程直接使用默认任务，无须创建额外线程。
 */
void RM_RTOS_Threads_Init(void) {
}

/**
 * @brief 在默认任务中轮询四类测量数据，并约每 100 ms 打印一次。
 * @param argument RTOS 任务参数，未使用。
 * @note 每次请求后休眠 2 ms，为 CAN 发送和反馈留出时间；数据由 CAN 回调自动更新。
 */
void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    osDelay(500);  // 等待模块上电启动。
    while (!dm_imu->SetActiveMode(false))
        osDelay(10);
    osDelay(10);

    uint32_t last_print = osKernelGetTickCount();
    while (true) {
        // set_cursor(0, 0);
        // clear_screen();

        dm_imu->getAccel();
        osDelay(2);
        dm_imu->getGyro();
        osDelay(2);
        dm_imu->getEuler();
        osDelay(2);
        dm_imu->getQuaternion();
        osDelay(2);

        if (osKernelGetTickCount() - last_print >= 100) {
            last_print = osKernelGetTickCount();
            print(
                "DM-IMU %s, Temp [C]: %.0f\r\n"
                "Accel [m/s^2]: %.3f %.3f %.3f\r\n"
                "Gyro [rad/s]: %.3f %.3f %.3f\r\n"
                "Euler [rad]: yaw=%.3f pitch=%.3f roll=%.3f\r\n"
                "Quaternion [w x y z]: %.5f %.5f %.5f %.5f\r\n\r\n",
                dm_imu->IsOnline() ? "online" : "offline",
                dm_imu->Temp,
                dm_imu->INS_accel[0],
                dm_imu->INS_accel[1],
                dm_imu->INS_accel[2],
                dm_imu->INS_gyro[0],
                dm_imu->INS_gyro[1],
                dm_imu->INS_gyro[2],
                dm_imu->INS_angle[0],
                dm_imu->INS_angle[1],
                dm_imu->INS_angle[2],
                dm_imu->INS_quat[0],
                dm_imu->INS_quat[1],
                dm_imu->INS_quat[2],
                dm_imu->INS_quat[3]
            );
        }
    }
}
