#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "chassis.h"
#include "controller.h"
#include "bsp_buzzer.h"

static bsp::CAN *can1 = nullptr;
static remote::DBUS *dbus = nullptr;

static bsp::Buzzer *buzzer = nullptr;

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


static bsp::CAN *can = nullptr;
static control::MotorCANBase *fl_motor = nullptr;
static control::MotorCANBase *fr_motor = nullptr;
static control::MotorCANBase *bl_motor = nullptr;
static control::MotorCANBase *br_motor = nullptr;

static control::Chassis *chassis = nullptr;

static control::MotorPWMBase *flywheel_left = nullptr;
static control::MotorPWMBase *flywheel_right = nullptr;

static control::MotorCANBase *steering_motor = nullptr;

uint32_t last_timestamp = 0;

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

  control::MotorCANBase *gimbal_motors[] = {pitch_motor, yaw_motor, steering_motor};

  int i = 0;
  while (i < 5000 || !imu->DataReady()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
    osDelay(1);
    ++i;
  }

  buzzer->SingTone(bsp::BuzzerNote::La6M);
  imu->Calibrate();

  i = 0;
  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
    osDelay(1);
    ++i;
  }
  buzzer->SingTone(bsp::BuzzerNote::Silent);
  float pitch_ratio, yaw_ratio;
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;
  float pitch_diff, yaw_diff;



  while (true) {
    if( HAL_GetTick() - last_timestamp > 550){
      pitch_motor->SetOutput(0);
      yaw_motor->SetOutput(0);
      steering_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
      while(1){
        if( HAL_GetTick() - last_timestamp < 500) break;
        osDelay(50);
      }
    }
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V || dbus->swr != remote::DOWN) {
          break;
        }
        osDelay(10);
      }
    }
    pitch_curr = imu->INS_angle[2];
    yaw_curr = imu->INS_angle[0];
    if(dbus->swr == remote::UP){
      gimbal->TargetAbs(0, 0);
      gimbal->Update();
      pitch_target = pitch_curr;
      yaw_target = yaw_curr;
      control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
      osDelay(1);
      continue ;
    }
    pitch_ratio = dbus->mouse.y / 32767.0 * 7.5 / 7.0;
    yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
    pitch_ratio = dbus->ch3 / 18000.0 / 7.0;
    yaw_ratio = dbus->ch4 / 18000.0 / 7.0;
    pitch_target =
        clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                    gimbal_param->pitch_max_);
    yaw_target = clip<float>(yaw_target + yaw_ratio, -gimbal_param->yaw_max_,
                             gimbal_param->yaw_max_);



    pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
    yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

    if (-0.005 < pitch_diff && pitch_diff < 0.005) {
      pitch_diff = 0;
    }

    gimbal->TargetRel(pitch_diff, yaw_diff);

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
    osDelay(1);
  }
}


const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority =
                                             (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t chassisTaskHandle;

void chassisTask(void *arg){
  UNUSED(arg);

  osDelay(10000); // DBUS initialization needs time

  control::MotorCANBase *motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

  float yaw_theta_max_iout = 0;
  float yaw_theta_max_out = 10;
  float yaw_omega_max_iout = 4000; // 10000
  float yaw_omega_max_out = 12288;

  float yaw_theta_pid_param_[3] = {10, 0, 0.3};
  float yaw_omega_pid_param_[3] = {20, 5, 0};

  control::ConstrainedPID *yaw_theta_pid_ = new control::ConstrainedPID(
      yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
  control::ConstrainedPID *yaw_omega_pid_ = new control::ConstrainedPID(
      yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);

  //float last_speed = 0;

  while (true) {
    if( HAL_GetTick() - last_timestamp > 550){

      chassis->SetSpeed(0, 0, 0);
      chassis->Update(false,30,20,60);
      fl_motor->SetOutput(0);
      fr_motor->SetOutput(0);
      bl_motor->SetOutput(0);
      br_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(motors, 4);
      while(1){
        if( HAL_GetTick() - last_timestamp < 500) break;
        osDelay(50);
      }
    }
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V || dbus->swr != remote::DOWN) {
          break;
        }
        osDelay(10);
      }
    }
    relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
    float yt_diff = wrap<float>(relative_angle, -PI, PI);
    float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
    float yo_in = yaw_motor->GetOmegaDelta(yt_out);
    float yo_out = yaw_omega_pid_->ComputeOutput(yo_in);
    yo_out = clip<float>(yo_out, -12288, 12288);

    chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);

    chassis->Update(false, 30, 20, 60);
    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(10);
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart6);

  can1 = new bsp::CAN(&hcan2, 0x201, false);
  dbus = new remote::DBUS(&huart3);
  buzzer = new bsp::Buzzer(&htim4, 3, 1000000);

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

  pitch_motor = new control::Motor6020(can1, 0x206);
  yaw_motor = new control::Motor6020(can1, 0x205);
  steering_motor = new control::Motor2006(can1, 0x207);
  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_FORTRESS;
  gimbal = new control::Gimbal(gimbal_data);
  gimbal_param = gimbal->GetData();

  can = new bsp::CAN(&hcan1, 0x201, true);
  fl_motor = new control::Motor3508(can, 0x202);
  fr_motor = new control::Motor3508(can, 0x201);
  bl_motor = new control::Motor3508(can, 0x203);
  br_motor = new control::Motor3508(can, 0x204);

  control::MotorCANBase *motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);

  flywheel_left = new control::MotorPWMBase(&htim1, 1, 1000000,500,1080);
  flywheel_right = new control::MotorPWMBase(&htim1, 2, 1000000,500,1080);
  flywheel_left->SetOutput(0);
  flywheel_right->SetOutput(0);
}



void KillAll() {
  control::MotorCANBase *motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
  control::MotorCANBase *gimbal_motors[] = {pitch_motor, yaw_motor};

  RM_EXPECT_TRUE(false, "Operation killed\r\n");
  while (true) {
    last_timestamp = dbus->timestamp;
    if ((HAL_GetTick() - last_timestamp) < 500 && (dbus->keyboard.bit.V || dbus->swr != remote::DOWN)) {
      break;
    }

    fl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    br_motor->SetOutput(0);
    pitch_motor->SetOutput(0);
    yaw_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(10);
  }
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
}

void RM_RTOS_Default_Task(const void *arg) {
  UNUSED(arg);

  while (true) {
    last_timestamp = dbus->timestamp;
    if( HAL_GetTick() - last_timestamp > 550)
      KillAll();
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN)
      KillAll();


    osDelay(200);
  }
}
