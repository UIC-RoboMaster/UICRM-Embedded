/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"

static bsp::CAN* can1 = nullptr;
static remote::DBUS* dbus = nullptr;

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
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

static IMU* imu = nullptr;

void imuTask(void* arg) {
    UNUSED(arg);

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            imu->Update();
        }
    }
}

static driver::MotorCANBase* pitch_motor = nullptr;
static driver::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;

const control::gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 4.3703f,
    .yaw_offset_ = 4.2062f,
    .pitch_max_ = 0.5571f,
    .yaw_max_ = PI/2,
};

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

void gimbalTask(void* arg) {
    UNUSED(arg);

    driver::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor};

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
        driver::MotorCANBase::TransmitOutput(gimbal_motors, 2);
        osDelay(1);
        ++i;
    }

    print("Start Calibration.\r\n");
    imu->Calibrate();

    i = 0;
    while (!imu->DataReady() || !imu->CaliDone()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        driver::MotorCANBase::TransmitOutput(gimbal_motors, 2);
        osDelay(1);
        ++i;
    }

    print("Gimbal Begin!\r\n");

    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    float pitch_diff, yaw_diff;
    float pitch_target = 0, yaw_target = 0;

    while (true) {
        if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
            while (true) {
                if (dbus->keyboard.bit.V) {
                    break;
                }
                osDelay(10);
            }
        }

        pitch_ratio = dbus->mouse.y / 32767.0 * 7.5 / 7.0;
        yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
        pitch_ratio = dbus->ch3 / 18000.0 / 7.0;
        yaw_ratio = -dbus->ch2 / 18000.0 / 7.0;
        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = clip<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = wrap<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        pitch_curr = imu->INS_angle[2];
        yaw_curr = imu->INS_angle[0];



        gimbal->TargetRel(pitch_diff, yaw_diff);

        gimbal->UpdateIMU(pitch_curr,yaw_curr);
        driver::MotorCANBase::TransmitOutput(gimbal_motors, 2);
        osDelay(1);
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart6);

    can1 = new bsp::CAN(&hcan1, 0x201, true);
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

    pitch_motor = new driver::Motor6020(can1, 0x206);
    yaw_motor = new driver::Motor6020(can1, 0x205);
    control::gimbal_t gimbal_data;
    gimbal_data.data = gimbal_init_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    control::gimbal_pid_t gimbalBasicPID;
    {
        float pitch_theta_max_iout = 0;
        float pitch_theta_max_out = 10;
        float pitch_omega_max_iout = 10000;
        float pitch_omega_max_out = 30000;
        float yaw_theta_max_iout = 0;
        float yaw_theta_max_out = 10;
        float yaw_omega_max_iout = 5000;  // 10000
        float yaw_omega_max_out = 30000;
        float* pitch_theta_pid_param = new float[3]{15, 0, 0};
        float* pitch_omega_pid_param = new float[3]{1000, 100, 0};
        float* yaw_theta_pid_param = new float[3]{15, 0, 0};
        float* yaw_omega_pid_param = new float[3]{1000, 100, 0};
        gimbalBasicPID.pitch_theta_pid = new control::ConstrainedPID(
            pitch_theta_pid_param, pitch_theta_max_iout, pitch_theta_max_out);
        gimbalBasicPID.pitch_omega_pid = new control::ConstrainedPID(
            pitch_omega_pid_param, pitch_omega_max_iout, pitch_omega_max_out);
        gimbalBasicPID.yaw_theta_pid =
            new control::ConstrainedPID(yaw_theta_pid_param, yaw_theta_max_iout, yaw_theta_max_out);
        gimbalBasicPID.yaw_omega_pid =
            new control::ConstrainedPID(yaw_omega_pid_param, yaw_omega_max_iout, yaw_omega_max_out);
    }
    gimbal_data.pid = gimbalBasicPID;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void KillAll() {
    driver::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor};

    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        if (dbus->keyboard.bit.V) {
            break;
        }
        pitch_motor->SetOutput(0);
        yaw_motor->SetOutput(0);
        driver::MotorCANBase::TransmitOutput(gimbal_motors, 2);
        osDelay(10);
    }
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (true) {
        if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN)
            KillAll();

        set_cursor(0, 0);
        clear_screen();

        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              imu->CaliDone() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", imu->Temp);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
              imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);

        print("\r\n");

        print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2,
              dbus->ch3);
        print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);

        osDelay(100);
    }
}
