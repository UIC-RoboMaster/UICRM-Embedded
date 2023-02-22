#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"

static bsp::CAN *can2 = nullptr;
static remote::DBUS *dbus = nullptr;

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

static control::MotorCANBase *pitch_motor = nullptr;
static control::MotorCANBase *yaw_motor = nullptr;
static control::Gimbal *gimbal = nullptr;
static control::gimbal_data_t *gimbal_param = nullptr;

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority =
                                                (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

void gimbalTask(void *arg) {
  UNUSED(arg);

  control::MotorCANBase *gimbal_motors[] = {pitch_motor, yaw_motor};

  print("Wait for beginning signal...\r\n");

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  int i = 0;
  while (i < 2000 || !imu->DataReady()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 2);
    osDelay(1);
    ++i;
  }
  i = 0;
  while (i < 5000) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 2);
    osDelay(1);
    ++i;
  }
  print("Start Calibration.\r\n");
  imu->Calibrate();

  i = 0;
  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 2);
    osDelay(1);
    ++i;
  }

  print("Gimbal Begin!\r\n");

  float pitch_ratio = 0, yaw_ratio = 0;
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;
  float pitch_diff, yaw_diff;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) {
          break;
        }
        osDelay(10);
      }
    }

    pitch_ratio = -dbus->mouse.y / 32767.0 * 7.5 / 7.0;
    yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
    pitch_ratio += -dbus->ch3 / 18000.0 / 7.0;
    yaw_ratio += -dbus->ch2 / 18000.0 / 7.0;
    pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                               gimbal_param->pitch_max_);
    yaw_target = clip<float>(yaw_target + yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

    pitch_curr = imu->INS_angle[2];
    yaw_curr = imu->INS_angle[0];

    pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
    yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

    if (-0.005 < pitch_diff && pitch_diff < 0.005) {
      pitch_diff = 0;
    }
    if(-0.005 < yaw_diff && yaw_diff < 0.005) {
      yaw_diff = 0;
    }

    gimbal->TargetRel(pitch_diff / 8, yaw_diff / 10);

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 2);
    osDelay(1);
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart6);

  can2 = new bsp::CAN(&hcan2, 0x201, false);
  dbus = new remote::DBUS(&huart3);

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
  imu = new IMU(imu_init, false);

  pitch_motor = new control::Motor6020(can2, 0x206);
  yaw_motor = new control::Motor6020(can2, 0x205);
  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_FORTRESS;
  gimbal = new control::Gimbal(gimbal_data);
  gimbal_param = gimbal->GetData();
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void KillAll() {
  control::MotorCANBase *gimbal_motors[] = {yaw_motor, pitch_motor};

  RM_EXPECT_TRUE(false, "Operation killed\r\n");
  while (true) {
    if (dbus->keyboard.bit.V) {
      break;
    }
    pitch_motor->SetOutput(0);
    yaw_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(gimbal_motors, 2);
    osDelay(10);
  }
}

void RM_RTOS_Default_Task(const void *arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN)
      KillAll();

    set_cursor(0, 0);
    clear_screen();

    print("# %.2f s, IMU %s\r\nTemp: %.2f\r\nEuler Angles: %.2f, %.2f, "
          "%.2f\r\nIs Calibrated: %s\r\nCH0: %-4d CH1: %-4d CH2: %-4d CH3: "
          "%-4d SWL: %d SWR: %d @ %d ms\r\n",
          HAL_GetTick() / 1000.0,
          imu->DataReady() ? "\033[1;42mReady\033[0m"
                           : "\033[1;41mNot Ready\033[0m",
          imu->Temp, imu->INS_angle[0] / PI * 180, imu->INS_angle[1] / PI * 180,
          imu->INS_angle[2] / PI * 180,
          imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m",
          dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr,
          dbus->timestamp);
    pitch_motor->PrintData();
    yaw_motor->PrintData();
    osDelay(100);
  }
}
