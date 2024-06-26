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

#include "QEKF.h"

#include "BMI088.h"
#include "IST8310.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "heater.h"
#include "main.h"

#define RX_SIGNAL (1 << 0)

static control::QEKF* ahrs = nullptr;
static driver::Heater* heater = nullptr;
static bsp::PWM* heater_pwm = nullptr;

static bsp::SPI* spi1 = nullptr;
static bsp::SPIMaster* spi1_master = nullptr;
static imu::BMI088* bmi088 = nullptr;
static bsp::GPIO* bmi088_accel_cs = nullptr;
static bsp::GPIO* bmi088_gyro_cs = nullptr;
static bsp::GPIT* bmi088_accel_int = nullptr;
static bsp::GPIT* bmi088_gyro_int = nullptr;

void BMI088ReceiveDone() {
    ahrs->Update(bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2], bmi088->accel_[0],
                 bmi088->accel_[1], bmi088->accel_[2]);
    heater->Update(bmi088->temperature_);
}

void RM_RTOS_Init(void) {
    DWT_Init(168);
    HAL_Delay(500);
    print_use_uart(&huart1, true, 921600);
    bsp::spi_init_t spiInit = {
        .hspi = &hspi1,
        .mode = bsp::SPI_MODE_DMA,
    };
    spi1 = new bsp::SPI(spiInit);
    bsp::spi_master_init_t spiMasterInit = {
        .spi = spi1,
    };
    spi1_master = new bsp::SPIMaster(spiMasterInit);
    bmi088_accel_cs = new bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
    bmi088_gyro_cs = new bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    bmi088_accel_int = new bsp::GPIT(INT1_ACCEL_Pin);
    bmi088_gyro_int = new bsp::GPIT(INT1_GYRO_Pin);
    imu::BMI088_init_t bmi088Init = {
        .spi_master = spi1_master,
        .CS_ACCEL = bmi088_accel_cs,
        .CS_GYRO = bmi088_gyro_cs,
        .INT_ACCEL = bmi088_accel_int,
        .INT_GYRO = bmi088_gyro_int,
        .is_DMA = true,
    };
    bmi088 = new imu::BMI088(bmi088Init);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    ahrs = new control::QEKF(DWT_GetDeltaT);
    heater_pwm = new bsp::PWM(&htim10, 1, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 45.0f,
    };
    heater = new driver::Heater(heater_init);
}

void RM_RTOS_Threads_Init(void) {
}
void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    bmi088->RegisterCallback(BMI088ReceiveDone);
    print("IMU Initialized!\r\n");
    osDelay(3000);

    while (bmi088->temperature_ < 44.5f) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", bmi088->temperature_);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", bmi088->accel_[0], bmi088->accel_[1],
              bmi088->accel_[2]);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", bmi088->gyro_[0], bmi088->gyro_[1],
              bmi088->gyro_[2]);
        print("\r\nTime Stamp: %.2f us\r\n", bmi088->time_);
        print("Calibrated: false\r\n");
        osDelay(10);
    }

    ahrs->Cailbrate();
    while (!ahrs->IsCailbrated()) {
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %10.4f\r\n", bmi088->temperature_);
        print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", bmi088->accel_[0], bmi088->accel_[1],
              bmi088->accel_[2]);
        print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", bmi088->gyro_[0], bmi088->gyro_[1],
              bmi088->gyro_[2]);
        print("\r\nTime Stamp: %.2f us\r\n", bmi088->time_);
        print("Calibrating...\r\n");
        osDelay(10);
    }

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print(
            "Temp: %10.4f\r\n"
            "Calibrated: true\r\n"
            "Yaw: %.2f Pitch: %.2f Roll: %.2f\r\n",
            bmi088->temperature_, ahrs->INS_angle[0] / PI * 180, ahrs->INS_angle[1] / PI * 180,
            ahrs->INS_angle[2] / PI * 180);
        osDelay(50);
    }
}
