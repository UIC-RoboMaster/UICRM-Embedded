#include "main.h"

#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "cmsis_os.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

static bsp::MPU6500* imu;

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim2);
    print_use_uart(&PRING_UART);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
    imu = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);

    print("IMU Initialized!\r\n");
    osDelay(3000);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", imu->temp);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", imu->acce.x, imu->acce.y, imu->acce.z);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", imu->gyro.x, imu->gyro.y,
              imu->gyro.z);
        print("MAG_X: %9.0f MAG_Y: %9.0f MAG_Z: %9.0f\r\n", imu->mag.x, imu->mag.y, imu->mag.z);
        print("\r\nTime Stamp: %lu us\r\n", imu->timestamp);
        osDelay(100);
    }
}
