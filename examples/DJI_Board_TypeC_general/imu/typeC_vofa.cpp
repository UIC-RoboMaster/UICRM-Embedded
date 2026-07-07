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

#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "vofa.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {
    .name = "imuTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
  public:
    using bsp::IMU_typeC::IMU_typeC;

  protected:
    void RxCompleteCallback() final {
        osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
    }
};

static IMU* imu = nullptr;
static driver::Vofa vofa;

static float ch_yaw = 0.f;
static float ch_pitch = 0.f;
static float ch_roll = 0.f;
static float ch_accel_x = 0.f;
static float ch_accel_y = 0.f;
static float ch_accel_z = 0.f;
static float ch_gyro_x = 0.f;
static float ch_gyro_y = 0.f;
static float ch_gyro_z = 0.f;
static float ch_temp = 0.f;

static const float* ImuChannels[] = {
    &ch_yaw,
    &ch_pitch,
    &ch_roll,
    &ch_accel_x,
    &ch_accel_y,
    &ch_accel_z,
    &ch_gyro_x,
    &ch_gyro_y,
    &ch_gyro_z,
    &ch_temp,
};

static void updateVofaChannels() {
    ch_yaw = imu->INS_angle[0] / PI * 180.f;
    ch_pitch = imu->INS_angle[1] / PI * 180.f;
    ch_roll = imu->INS_angle[2] / PI * 180.f;
    ch_accel_x = imu->INS_accel[0];
    ch_accel_y = imu->INS_accel[1];
    ch_accel_z = imu->INS_accel[2];
    ch_gyro_x = imu->INS_gyro[0];
    ch_gyro_y = imu->INS_gyro[1];
    ch_gyro_z = imu->INS_gyro[2];
    ch_temp = imu->Temp;
}

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            imu->Update();
            updateVofaChannels();
            vofa.Send();
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);
    HAL_Delay(100);

    vofa.Attach(print_uart);
    vofa.BindChannels(ImuChannels, sizeof(ImuChannels) / sizeof(ImuChannels[0]));

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
    imu = new IMU(imu_init, true);
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    imu->Calibrate();
    while (true)
        osDelay(1000);
}
