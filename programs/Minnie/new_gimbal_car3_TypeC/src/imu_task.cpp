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

#include "imu_task.h"

osThreadId_t IMUTaskHandle;

#define RX_SIGNAL (1 << 0)

control::AHRS* ahrs = nullptr;
driver::Heater* heater = nullptr;
bsp::PWM* heater_pwm = nullptr;

bsp::SPI* spi1 = nullptr;
bsp::SPIMaster* spi1_master = nullptr;
imu::BMI088* bmi088 = nullptr;
bsp::GPIO* bmi088_accel_cs = nullptr;
bsp::GPIO* bmi088_gyro_cs = nullptr;
bsp::GPIT* bmi088_accel_int = nullptr;
bsp::GPIT* bmi088_gyro_int = nullptr;

bsp::I2C* i2c3 = nullptr;
bsp::GPIO* ist8310_rst = nullptr;
bsp::GPIT* ist8310_int = nullptr;
imu::IST8310* ist8310 = nullptr;

void IMU_print() {
    print("IMU data:\r\n");
    print("gyro: x:%.2f, y:%.2f, z:%.2f\r\n", bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2]);
    print("accel: x:%.2f, y:%.2f, z:%.2f\r\n", bmi088->accel_[0], bmi088->accel_[1],
          bmi088->accel_[2]);
    print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
          ahrs->INS_angle[2]);
}

void BMI088ReceiveDone() {
    ahrs->Update(bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2], bmi088->accel_[0],
                 bmi088->accel_[1], bmi088->accel_[2]);
    heater->Update(bmi088->temperature_);
}

void IMU_Init() {
    HAL_Delay(500);
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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    ist8310_rst = new bsp::GPIO(GPIOG, GPIO_PIN_6);
    ist8310_int = new bsp::GPIT(DRDY_IST8310_Pin);
    bsp::i2c_init_t i2c3_init = {.hi2c = &hi2c3, .mode = bsp::I2C_MODE_IT};
    i2c3 = new bsp::I2C(i2c3_init);
    imu::IST8310_init_t ist8310Init = {.i2c = i2c3, .drdy = ist8310_int, .rst = ist8310_rst};
    ist8310 = new imu::IST8310(ist8310Init);

    ahrs = new control::AHRS(false);
    heater_pwm = new bsp::PWM(&htim10, 1, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 45.0f,
    };
    heater = new driver::Heater(heater_init);

    bmi088->RegisterCallback(BMI088ReceiveDone);
    osDelay(500);
}


