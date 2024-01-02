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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "wit_protocol.h"

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

/// The class IUART is a wrapper of bsp::UART, which is used to
/// implement the virtual function RxCompleteCallback() to
/// receive data from the wit-imu
class IUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    void RxCompleteCallback() final {
        osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
    }
};

static IUART* wituart = nullptr;

/// The class WITUART is not an UART, It means that the WIT-IMU using UART
static imu::WITUART* witimu = nullptr;

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            witimu->Update();
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart8);
    wituart = new IUART(&huart6);
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
    /// Calibrate the IMU
    status_data[0] = 0x01;
    status_data[1] = 0x00;
    witimu->WriteReg(0x01, status_data);
    HAL_Delay(100);
    /// Lock the IMU
    witimu->Lock();
    HAL_Delay(100);
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        // print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0);
        // print("Temp: %.2f\r\n", witimu->temp_);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", witimu->INS_angle[0] / PI * 180,
              witimu->INS_angle[1] / PI * 180, witimu->INS_angle[2] / PI * 180);

        osDelay(50);
    }
}
