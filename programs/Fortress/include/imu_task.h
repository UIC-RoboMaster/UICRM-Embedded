#pragma once
#include "main.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "spi.h"
#include "bsp_imu.h"
#define RX_SIGNAL (1 << 0)
extern osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};

class IMU : public bsp::IMU_typeC {
  public:
    using bsp::IMU_typeC::IMU_typeC;

  protected:
    void RxCompleteCallback() final;
};
extern IMU* imu;
void imuTask(void* arg);

void init_imu();