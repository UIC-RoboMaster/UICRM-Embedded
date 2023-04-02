#pragma once
#include "main.h"

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "protocol.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
extern osThreadId_t refereeTaskHandle;

class CustomUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final;
};

extern CustomUART* referee_uart;
extern communication::Referee* referee;

void refereeTask(void* arg);
void init_referee();