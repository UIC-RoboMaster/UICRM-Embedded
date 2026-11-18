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
#include "BMI088.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "heater.h"
#include "main.h"
#include "tim.h"

static bsp::SPI* spi2 = nullptr;
static bsp::SPIMaster* spi2_master = nullptr;
static imu::BMI088* bmi088 = nullptr;
static bsp::GPIO* bmi088_accel_cs = nullptr;
static bsp::GPIO* bmi088_gyro_cs = nullptr;
static bsp::GPIT* bmi088_accel_int = nullptr;
static bsp::GPIT* bmi088_gyro_int = nullptr;

static control::AHRS* ahrs = nullptr;
static driver::Heater* heater = nullptr;
static bsp::PWM* heater_pwm = nullptr;

static volatile bool data_ready = false;

static void BMI088ReceiveDone() {
    data_ready = true;
    ahrs->Update(
        bmi088->gyro_[0],
        bmi088->gyro_[1],
        bmi088->gyro_[2],
        bmi088->accel_[0],
        bmi088->accel_[1],
        bmi088->accel_[2]
    );
    heater->Update(bmi088->temperature_);
}

void RM_RTOS_Init(void) {
    print_use_rtt();

    // 板载 BMI088：SPI2 + PC0(Accel CS) / PC3(Gyro CS)，数据就绪中断 PE10 / PE12
    bsp::spi_init_t spiInit = {
        .hspi = &hspi2,
        .mode = bsp::SPI_MODE_DMA,
    };
    spi2 = new bsp::SPI(spiInit);
    bsp::spi_master_init_t spiMasterInit = {
        .spi = spi2,
    };
    spi2_master = new bsp::SPIMaster(spiMasterInit);
    bmi088_accel_cs = new bsp::GPIO(Accel_CS_GPIO_Port, Accel_CS_Pin);
    bmi088_gyro_cs = new bsp::GPIO(Gyro_CS_GPIO_Port, Gyro_CS_Pin);
    bmi088_accel_int = new bsp::GPIT(Accel_INT_Pin);
    bmi088_gyro_int = new bsp::GPIT(Gyro_INT_Pin);

    imu::BMI088_init_t bmi088Init = {
        .spi_master = spi2_master,
        .CS_ACCEL = bmi088_accel_cs,
        .CS_GYRO = bmi088_gyro_cs,
        .INT_ACCEL = bmi088_accel_int,
        .INT_GYRO = bmi088_gyro_int,
        .is_DMA = true,
    };
    bmi088 = new imu::BMI088(bmi088Init);

    ahrs = new control::AHRS(false);

    // 板载 IMU 加热电阻：TIM3_CH4，目标 45℃
    heater_pwm = new bsp::PWM(&BOARD_TIM_IMUHEATER, BOARD_TIM_IMUHEATER_CH, 1000000, 500, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 45.0f,
        .pid_param = new float[3]{40.0f, 0.01f, 0.0f},
        .heater_I_limit = 30.0f,
        .heater_output_limit = 100.0f,
    };
    heater = new driver::Heater(heater_init);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    bmi088->RegisterCallback(BMI088ReceiveDone);
    osDelay(3000);

    // 恒温后再标定，与 C 板 IMU 的流程一致；加热异常时最多等 20s 后仍继续
    uint32_t start = HAL_GetTick();
    while (bmi088->temperature_ < 43.0f && HAL_GetTick() - start < 20000) {
        set_cursor(0, 0);
        clear_screen();
        print("IMU warming up... Temp: %.2f\r\n", bmi088->temperature_);
        print("Accel X/Y/Z: %.3f %.3f %.3f\r\n", bmi088->accel_[0], bmi088->accel_[1], bmi088->accel_[2]);
        osDelay(100);
    }

    ahrs->Cailbrate();
    while (!ahrs->IsCailbrated()) {
        set_cursor(0, 0);
        clear_screen();
        print("Calibrating IMU, keep the robot still...\r\n");
        print("Temp: %.2f\r\n", bmi088->temperature_);
        osDelay(100);
    }

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print(
            "# %.2f s, IMU %s\r\n",
            HAL_GetTick() / 1000.0,
            data_ready ? "Ready" : "Not Ready"
        );
        print("Temp: %.2f\r\n", bmi088->temperature_);
        print(
            "Euler Angles(Yaw/Pitch/Roll): %.2f, %.2f, %.2f\r\n",
            ahrs->INS_angle[0] / PI * 180,
            ahrs->INS_angle[1] / PI * 180,
            ahrs->INS_angle[2] / PI * 180
        );

        print("Accel X: %.3f m/s^2\r\n", bmi088->accel_[0]);
        print("Accel Y: %.3f m/s^2\r\n", bmi088->accel_[1]);
        print("Accel Z: %.3f m/s^2\r\n", bmi088->accel_[2]);

        print("Gyro X: %.3f deg/s\r\n", bmi088->gyro_[0] / PI * 180);
        print("Gyro Y: %.3f deg/s\r\n", bmi088->gyro_[1] / PI * 180);
        print("Gyro Z: %.3f deg/s\r\n", bmi088->gyro_[2] / PI * 180);

        print("Is Calibrated: %s\r\n", ahrs->IsCailbrated() ? "Yes" : "No");
        osDelay(50);
    }
}
