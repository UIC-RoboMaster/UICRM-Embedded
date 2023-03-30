#include "main.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "remote_task.h"
#include "shoot_task.h"

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
    print_use_uart(&huart6);
    init_buzzer();
    init_remote();
    init_imu();
    init_shoot();
    init_gimbal();
    init_chassis();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(1000);
    while (true) {
        osDelay(50);
    }
}
