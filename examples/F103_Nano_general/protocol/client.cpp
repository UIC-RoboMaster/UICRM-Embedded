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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"
#include "protocol.h"
#include "rgb.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

const osThreadAttr_t clientTaskAttribute = {.name = "clientTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t clientTaskHandle;

class RefereeUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final {
        osThreadFlagsSet(clientTaskHandle, RX_SIGNAL);
    }
};

static communication::Host* host = nullptr;
static RefereeUART* host_uart = nullptr;
static bsp::GPIO* led = nullptr;

void clientTask(void* arg) {
    UNUSED(arg);
    uint32_t length;
    uint8_t* data;

    while (true) {
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
            length = host_uart->Read(&data);
            host->Receive(communication::package_t{data, (int)length});
            led->Low();
            osDelay(200);
            led->High();
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart2, false);

    host_uart = new RefereeUART(&huart1);
    host_uart->SetupRx(300);
    host_uart->SetupTx(300);

    host = new communication::Host;

    led = new bsp::GPIO(LED_GPIO_Port, LED_Pin);
    led->High();
}

void RM_RTOS_Threads_Init(void) {
    clientTaskHandle = osThreadNew(clientTask, nullptr, &clientTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("%s", host->pack.chars);
        osDelay(100);
    }
}