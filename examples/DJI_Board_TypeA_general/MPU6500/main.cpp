/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_spi.h"
#include "MPU6500.h"
#include "cmsis_os.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

static driver::MPU6500* imu=nullptr;
static bsp::GPIO* imu_cs = nullptr;
static bsp::GPIT* mpu6500_it = nullptr;
static bsp::SPI* spi5 = nullptr;
static bsp::SPIMaster* spi5_master = nullptr;

void RM_RTOS_Init(void) {
    imu_cs = new bsp::GPIO(ONBOARD_IMU_CS_GROUP,ONBOARD_IMU_CS_PIN);
    mpu6500_it = new bsp::GPIT(MPU6500_IT_Pin);
    bsp::spi_init_t spi5_init{
        .hspi=&hspi5,
        .mode=bsp::SPI_MODE_DMA
    };
    spi5 = new bsp::SPI(spi5_init);
    bsp::spi_master_init_t spi5_master_init = {
        .spi=spi5,
    };
    spi5_master = new bsp::SPIMaster(spi5_master_init);
    driver::mpu6500_init_t mpu6500_init = {
        .spi=spi5_master,
        .cs=imu_cs,
        .int_pin=mpu6500_it,
        .use_mag=true,
        .dma=true,
    };
    imu = new driver::MPU6500(mpu6500_init);
    bsp::SetHighresClockTimer(&htim7);
    print_use_uart(&PRING_UART);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    print("IMU Initialized!\r\n");
    osDelay(3000);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", imu->temperature_);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", imu->accel_[0], imu->accel_[1], imu->accel_[2]);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", imu->gyro_[0], imu->gyro_[1],
              imu->gyro_[2]);
        print("MAG_X: %9.0f MAG_Y: %9.0f MAG_Z: %9.0f\r\n", imu->mag_[0], imu->mag_[1], imu->mag_[2]);
        print("\r\nTime Stamp: %.2f us\r\n", imu->time_);
        osDelay(100);
    }
}
