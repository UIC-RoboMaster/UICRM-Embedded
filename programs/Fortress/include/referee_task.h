#pragma once
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "dbus.h"
#include "main.h"
#include "protocol.h"

#define RX_SIGNAL (1 << 0)
extern osThreadId_t refereeTaskHandle;
const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

extern osThreadId_t refereercTaskHandle;
const osThreadAttr_t refereercTaskAttribute = {.name = "refereercTask",
                                               .attr_bits = osThreadDetached,
                                               .cb_mem = nullptr,
                                               .cb_size = 0,
                                               .stack_mem = nullptr,
                                               .stack_size = 256 * 4,
                                               .priority = (osPriority_t)osPriorityNormal,
                                               .tz_module = 0,
                                               .reserved = 0};

class RefereeUART : public bsp::UART {
  public:
    using bsp::UART::UART;
    RefereeUART(UART_HandleTypeDef* huart, osThreadId_t* rTaskHandle)
        : bsp::UART(huart), refereeTaskHandle_(rTaskHandle) {
    }

  protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final;

  private:
    osThreadId_t* refereeTaskHandle_;
};

extern RefereeUART* referee_uart;
extern communication::Referee* referee;
extern RefereeUART* refereerc_uart;
extern communication::Referee* refereerc;

void refereeTask(void* arg);
void refereercTask(void* arg);
void init_referee();