#include "main.h"


#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"


#include "buzzer.h"
#include "remote_task.h"
#include "imu_task.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
uint32_t last_timestamp = 0;

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
    print_use_uart(&huart6);
    init_buzzer();
    init_dbus();
    init_imu();
    init_shoot();
    init_gimbal();
    init_chassis();

}

void KillAll() {

    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        last_timestamp = dbus->timestamp;
        if ((HAL_GetTick() - last_timestamp) < 500 &&
            (dbus->keyboard.bit.V || dbus->swr != remote::DOWN)) {
            break;
        }

        osDelay(10);
    }
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(1000);
    while (true) {
        last_timestamp = dbus->timestamp;
        if (HAL_GetTick() - last_timestamp > 550)
            KillAll();
        if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN)
            KillAll();

        osDelay(200);
    }
}
