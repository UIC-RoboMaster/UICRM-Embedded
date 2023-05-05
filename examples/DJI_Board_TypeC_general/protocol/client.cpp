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
                                            .stack_size = 128 * 4,
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
static display::RGB* led = nullptr;

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
            led->Display(0xFF00FF00);
            osDelay(200);
            led->Display(0xFFFF0000);
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart6);

    host_uart = new RefereeUART(&huart1);
    host_uart->SetupRx(300);
    host_uart->SetupTx(300);

    host = new communication::Host;

    led = new display::RGB(&htim5, 3, 2, 1, 1000000);
    led->Display(0x00000000);
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