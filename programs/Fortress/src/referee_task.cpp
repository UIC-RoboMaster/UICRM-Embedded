#include "referee_task.h"

osThreadId_t refereeTaskHandle;

void CustomUART::RxCompleteCallback() {
    osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL);
}

CustomUART* referee_uart = nullptr;
communication::Referee* referee = nullptr;

void refereeTask(void* arg) {
    UNUSED(arg);
    uint32_t length;
    uint8_t* data;
    osDelay(2000);
    while (true) {
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
            length = referee_uart->Read(&data);
            referee->Receive(communication::package_t{data, (int)length});
        }
    }
}

void init_referee() {
    referee_uart = new CustomUART(&huart6);
    referee_uart->SetupRx(300);
    referee_uart->SetupTx(300);
    referee = new communication::Referee;
}
