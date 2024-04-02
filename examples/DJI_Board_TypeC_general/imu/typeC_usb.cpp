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
#include "protocol.h"
#include "spi.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
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

static communication::Host* mini_pc = nullptr;
static bsp::UART* mini_pc_uart = nullptr;

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            imu->Update();
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_usb();

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

    mini_pc_uart = new bsp::UART(&huart1);
    mini_pc_uart->SetBaudrate(921600);
    HAL_Delay(100);  // wait for the baudrate to take effect
    mini_pc_uart->SetupTx(300);
    mini_pc_uart->SetupRx(300);
    mini_pc = new communication::Host(mini_pc_uart);
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    imu->Calibrate();
    while (true) {
        for (uint8_t i = 0; i < 50; i++) {
            mini_pc->gimbal_current_status.current_imu_yaw = imu->INS_angle[0] / PI * 180;
            mini_pc->gimbal_current_status.current_imu_roll = imu->INS_angle[1] / PI * 180;
            mini_pc->gimbal_current_status.current_imu_pitch = imu->INS_angle[2] / PI * 180;
            mini_pc->Transmit(communication::GIMBAL_CURRENT_STATUS);
            osDelay(1);
        }
        set_cursor(0, 0);
        clear_screen();
        print(
            "# %.2f s, IMU %s\r\nTemp: %.2f\r\nEuler Angles: %.2f, %.2f, "
            "%.2f\r\nIs Calibrated: %s\r\n",
            HAL_GetTick() / 1000.0,
            imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m", imu->Temp,
            imu->INS_angle[0] / PI * 180, imu->INS_angle[1] / PI * 180,
            imu->INS_angle[2] / PI * 180,
            imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m");
    }
}
