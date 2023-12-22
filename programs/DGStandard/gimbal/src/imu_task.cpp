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

#include "imu_task.h"

#include "bsp_os.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

control::AHRS* ahrs = nullptr;
driver::Heater* heater = nullptr;
bsp::PWM* heater_pwm = nullptr;

imu::MPU6500* mpu6500 = nullptr;
bsp::GPIO* imu_cs = nullptr;
bsp::GPIT* mpu6500_it = nullptr;
bsp::SPI* spi5 = nullptr;
bsp::SPIMaster* spi5_master = nullptr;

osThreadId_t imuTaskHandle;

void MPU6500ReceiveDone() {
    osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
}

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            // ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2],
            // mpu6500->accel_[0], mpu6500->accel_[1], mpu6500->accel_[2], mpu6500->mag_[0],
            // mpu6500->mag_[1], mpu6500->mag_[2]);
            ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2],
                         mpu6500->accel_[0], mpu6500->accel_[1], mpu6500->accel_[2]);
            heater->Update(mpu6500->temperature_);
        }
    }
}

void init_imu() {
    imu_cs = new bsp::GPIO(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
    mpu6500_it = new bsp::GPIT(MPU6500_IT_Pin);
    bsp::spi_init_t spi5_init{.hspi = &hspi5, .mode = bsp::SPI_MODE_DMA};
    spi5 = new bsp::SPI(spi5_init);
    bsp::spi_master_init_t spi5_master_init = {
        .spi = spi5,
    };
    spi5_master = new bsp::SPIMaster(spi5_master_init);
    imu::mpu6500_init_t mpu6500_init = {
        .spi = spi5_master,
        .cs = imu_cs,
        .int_pin = mpu6500_it,
        .use_mag = true,
        .dma = true,
    };
    mpu6500 = new imu::MPU6500(mpu6500_init);
    bsp::SetHighresClockTimer(&htim7);

    ahrs = new control::AHRS(false);
    heater_pwm = new bsp::PWM(&htim3, 2, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 50.0f,
    };
    heater = new driver::Heater(heater_init);
    mpu6500->RegisterCallback(MPU6500ReceiveDone);
}