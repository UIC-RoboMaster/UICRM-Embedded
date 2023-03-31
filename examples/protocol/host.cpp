#include <cstring>

#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"
#include "protocol.h"
#include "rgb.h"

static communication::Host* host = nullptr;
static bsp::UART* client = nullptr;
static display::RGB* led = nullptr;

void RM_RTOS_Init(void) {
    client = new bsp::UART(&huart1);
    client->SetupTx(300);
    client->SetupRx(300);

    host = new communication::Host;

    led = new display::RGB(&htim5, 3, 2, 1, 1000000);
    led->Display(0x00000000);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    communication::package_t frame;
    communication::pack_t message[3] = {
        {"Sazabi Gundam!"}, {"Sinanju Gundam!"}, {"Kshatriya Gundam!"}};

    while (true) {
        for (int i = 0; i < 3; ++i) {
            memcpy(&host->pack, message + i, sizeof(communication::pack_t));
            frame = host->Transmit(communication::PACK);
            client->Write(frame.data, frame.length);
            led->Display(0xFF00FF00);
            osDelay(200);
            led->Display(0xFFFF0000);
            osDelay(300);
        }
    }
}