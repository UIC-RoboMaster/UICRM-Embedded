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
#include "public_port.h"
void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
    print_use_uart(&huart6);
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
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(1000);
    char s[50];
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        switch(remote_mode){
            case REMOTE_MODE_PREPARE:
                strcpy(s,"PREPARE");
                break;
                case REMOTE_MODE_STOP:
                        strcpy(s,"STOP");
                        break;
                case REMOTE_MODE_KILL:
                    strcpy(s,"KILL");
                    break;
                case REMOTE_MODE_MANUAL:
                        strcpy(s,"MANUAL");
                        break;
                case REMOTE_MODE_SPIN:
                        strcpy(s,"SPIN");
                        break;
                default:
                        strcpy(s,"UNKNOWN");
                        break;
        }
        print("Mode:%s\r\n",s);
        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\nSWL: %d SWR: %d "
            "TWL: %d "
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr, dbus->ch4,
            dbus->timestamp);
        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", imu->Temp);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
              imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);
        print("Is Calibrated: %s\r\n",
              imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m");
        osDelay(50);
    }
}
