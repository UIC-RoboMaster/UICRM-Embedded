#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority =
                                             (osPriority_t)osPriorityNormal,
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

static IMU *imu = nullptr;

void imuTask(void *arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags =
        osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) { // unnecessary check
      imu->Update();
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart6);

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
  imu = new IMU(imu_init, true);
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

void RM_RTOS_Default_Task(const void *arg) {
  UNUSED(arg);
  imu->Calibrate();
  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
          imu->DataReady() ? "\033[1;42mReady\033[0m"
                           : "\033[1;41mNot Ready\033[0m");
    print("Temp: %.2f\r\n", imu->Temp);
    print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
          imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);
    print("Is Calibrated: %s\r\n",
          imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m");
    osDelay(50);
  }
}
