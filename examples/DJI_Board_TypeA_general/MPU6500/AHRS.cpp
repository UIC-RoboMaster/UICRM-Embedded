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

#include "AHRS.h"

#include "MPU6500.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "heater.h"
#include "main.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

#define RX_SIGNAL (1 << 0)

static control::AHRS* ahrs = nullptr;
static driver::Heater* heater = nullptr;
static bsp::PWM* heater_pwm = nullptr;

static imu::MPU6500* mpu6500 = nullptr;
static bsp::GPIO* imu_cs = nullptr;
static bsp::GPIT* mpu6500_it = nullptr;
static bsp::SPI* spi5 = nullptr;
static bsp::SPIMaster* spi5_master = nullptr;

void MPU6500ReceiveDone() {
    // ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2],
    // mpu6500->accel_[0], mpu6500->accel_[1], mpu6500->accel_[2], mpu6500->mag_[0],
    // mpu6500->mag_[1], mpu6500->mag_[2]);
    ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2], mpu6500->accel_[0],
                 mpu6500->accel_[1], mpu6500->accel_[2]);
    heater->Update(mpu6500->temperature_);
}

void RM_RTOS_Init(void) {
    HAL_Delay(500);
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
    print_use_uart(&PRING_UART, true, 921600);

    ahrs = new control::AHRS(false);
    heater_pwm = new bsp::PWM(&htim3, 2, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 50.0f,
        .pid_param = new float[3]{160, 0.1, 0},
    };
    heater = new driver::Heater(heater_init);
}

void RM_RTOS_Threads_Init(void) {
}
void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    mpu6500->RegisterCallback(MPU6500ReceiveDone);
    print("IMU Initialized!\r\n");
    osDelay(3000);

    while (mpu6500->temperature_ < 43.0f) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", mpu6500->temperature_);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", mpu6500->accel_[0], mpu6500->accel_[1],
              mpu6500->accel_[2]);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", mpu6500->gyro_[0], mpu6500->gyro_[1],
              mpu6500->gyro_[2]);
        print("MAG_X: %9.0f MAG_Y: %9.0f MAG_Z: %9.0f\r\n", mpu6500->mag_[0], mpu6500->mag_[1],
              mpu6500->mag_[2]);
        print("\r\nTime Stamp: %.2f us\r\n", mpu6500->time_);
        print("Calibrated: false\r\n");
        osDelay(10);
    }

    ahrs->Cailbrate();
    while (!ahrs->IsCailbrated()) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", mpu6500->temperature_);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", mpu6500->accel_[0], mpu6500->accel_[1],
              mpu6500->accel_[2]);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", mpu6500->gyro_[0], mpu6500->gyro_[1],
              mpu6500->gyro_[2]);
        print("MAG_X: %9.0f MAG_Y: %9.0f MAG_Z: %9.0f\r\n", mpu6500->mag_[0], mpu6500->mag_[1],
              mpu6500->mag_[2]);
        print("\r\nTime Stamp: %.2f us\r\n", mpu6500->time_);
        print("Calibrating...\r\n");
        osDelay(10);
    }

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", mpu6500->temperature_);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", mpu6500->accel_[0], mpu6500->accel_[1],
              mpu6500->accel_[2]);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", mpu6500->gyro_[0], mpu6500->gyro_[1],
              mpu6500->gyro_[2]);
        print("MAG_X: %9.0f MAG_Y: %9.0f MAG_Z: %9.0f\r\n", mpu6500->mag_[0], mpu6500->mag_[1],
              mpu6500->mag_[2]);
        print("\r\nTime Stamp: %.2f us\r\n", mpu6500->time_);
        print("Calibrated: true\r\n");
        print("Yaw: %.2f Pitch: %.2f Roll: %.2f\r\n", ahrs->INS_angle[0] / PI * 180,
              ahrs->INS_angle[1] / PI * 180, ahrs->INS_angle[2] / PI * 180);
        osDelay(50);
    }
}
