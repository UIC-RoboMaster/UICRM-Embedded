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

namespace {
    ImuSample latest_sample;
}

osThreadId_t imuTaskHandle = nullptr;

void IMU::RxCompleteCallback() {
    if (imuTaskHandle != nullptr)
        osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
}

IMU* imu = nullptr;

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if ((flags & osFlagsError) == 0 && (flags & RX_SIGNAL) != 0) {
            const bool was_calibrated = imu->CaliDone();
            imu->Update();

            ImuSample next;
            next.sample_ms = HAL_GetTick();
            next.warmed_up = imu->DataReady();
            next.calibrated = imu->CaliDone();
            // CaliDone becomes true one update before gyro/attitude publication.
            next.valid = was_calibrated && next.warmed_up;
            if (next.valid) {
                for (int i = 0; i < 3; ++i) {
                    next.gyro[i] = imu->INS_gyro[i];
                    next.angle[i] = imu->INS_angle[i];
                }
            }
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            latest_sample = next;
            __set_PRIMASK(primask);
        }
    }
}

ImuSample get_imu_sample() {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const ImuSample sample = latest_sample;
    __set_PRIMASK(primask);
    return sample;
}

void init_imu() {
    bsp::IST8310_init_t IST8310_init;
    IST8310_init.hi2c = &hi2c3;
    IST8310_init.int_pin = DRDY_IST8310_Pin;
    IST8310_init.rst_group = GPIOG;
    IST8310_init.rst_pin = GPIO_PIN_6;
    bsp::BMI088_init_t BMI088_init;
    BMI088_init.hspi = &hspi1;
    BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
    BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
    BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
    BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;
    bsp::heater_init_t heater_init;
    heater_init.htim = &htim10;
    heater_init.channel = 1;
    heater_init.clock_freq = 1000000;
    heater_init.temp = 45;
    bsp::IMU_typeC_init_t imu_init;
    imu_init.IST8310 = IST8310_init;
    imu_init.BMI088 = BMI088_init;
    imu_init.heater = heater_init;
    imu_init.hspi = &hspi1;
    imu_init.hdma_spi_rx = &hdma_spi1_rx;
    imu_init.hdma_spi_tx = &hdma_spi1_tx;
    imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
    imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;
    imu = new IMU(imu_init, false);
}
