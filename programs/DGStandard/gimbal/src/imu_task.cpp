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
#include "bsp_uart.h"

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

bsp::UART* wituart = nullptr;

/// The class WITUART is not an UART, It means that the WIT-IMU using UART
imu::WITUART* witimu = nullptr;

osThreadId_t extimuTaskHandle;

float yaw_offset = 0;
bool imu_ok = false;

void MPU6500ReceiveDone() {
    ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2],
                 mpu6500->accel_[0], mpu6500->accel_[1], mpu6500->accel_[2]);
    heater->Update(mpu6500->temperature_);
}


void extimuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            witimu->Update();
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

    HAL_Delay(500);
    wituart = new bsp::UART(&huart6);
    /// Some models of wit-imu may need to change baudrate to 921600
    wituart->SetBaudrate(921600);
    /// Setup Rx and Tx buffer size
    wituart->SetupTx(6);
    wituart->SetupRx(12);
    witimu = new imu::WITUART(wituart);
    /// Before write the register, you need to unlock the wit-imu
    witimu->Unlock();
    HAL_Delay(100);
    /// Set only output the INS data (Euler Angles)
    uint8_t status_data[] = {0x08, 0x00};
    witimu->WriteReg(0x02, status_data);
    HAL_Delay(100);
    // Use 6-axis mode
    status_data[0] = 0x01;
    status_data[1] = 0x00;
    witimu->WriteReg(0x24, status_data);
    HAL_Delay(100);
    /// Calibrate the IMU
    //    status_data[0] = 0x01;
    //    status_data[1] = 0x00;
    //    witimu->WriteReg(0x01, status_data);
    //    HAL_Delay(5000);
    //    status_data[0] = 0x01;
    //    status_data[1] = 0x00;
    //    witimu->WriteReg(0x61, status_data);
    //    HAL_Delay(3000);
    /// Lock the IMU
    witimu->Lock();
    HAL_Delay(100);
}
void reset_yaw() {
    //    yaw_offset = witimu->INS_angle[2];
    imu_ok = true;
}
