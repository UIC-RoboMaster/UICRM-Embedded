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

class IUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    void RxCompleteCallback() final {
        osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
    }
};

static IUART* wituart = nullptr;

static imu::WITUART* witimu = nullptr;

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            witimu->Update();
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_usb();
    wituart = new IUART(&huart1);
    //    wituart->SetBaudrate(921600);
    wituart->SetupTx(100);
    wituart->SetupRx(100);
    witimu = new imu::WITUART(wituart);
    uint8_t status_data[] = {0x08, 0x00};
    witimu->Unlock();
    HAL_Delay(100);
    witimu->WriteReg(0x02, status_data);
    HAL_Delay(100);
    status_data[0] = 0x01;
    status_data[1] = 0x00;
    witimu->WriteReg(0x01, status_data);
    HAL_Delay(100);
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
        print("Euler Angles: %.2f, %.2f, %.2f\r\n, witimu->INS_angle[0] / PI * 180",
              witimu->INS_angle[1] / PI * 180, witimu->INS_angle[2] / PI * 180);

        osDelay(50);
    }
}
