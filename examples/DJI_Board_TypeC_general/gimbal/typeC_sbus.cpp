/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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
#include "gimbal.h"
#include "i2c.h"
#include "main.h"
#include "sbus.h"
#include "spi.h"

static bsp::CAN* can1 = nullptr;
static remote::SBUS* sbus = nullptr;

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

const control::gimbal_data_t gimbal_init_data = {.pitch_offset_ = 2.8582f,
                                                 .yaw_offset_ = 2.5840f,
                                                 .pitch_max_ = 0.4897f,
                                                 .yaw_max_ = PI / 2,
                                                 .yaw_circle_ = true};

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

    print("Wait for beginning signal...\r\n");
    while (true) {
        if (sbus->ch7 >= 0) {
            break;
        }
        osDelay(100);
    }

    int i = 0;
    while (i < 2000 || !imu->DataReady()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
        ++i;
    }

    print("Start Calibration.\r\n");
    imu->Calibrate();

    i = 0;
    while (!imu->DataReady() || !imu->CaliDone()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
        ++i;
    }

    print("Gimbal Begin!\r\n");

    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    float pitch_diff, yaw_diff;
    float pitch_target = 0, yaw_target = 0;

    while (true) {
        if (sbus->ch7 < 0) {
            while (true) {
                if (sbus->ch7 >= 0) {
                    break;
                }
                osDelay(10);
            }
        }

        pitch_ratio = sbus->ch3 / 18000.0 / 7.0;
        yaw_ratio = -sbus->ch4 / 18000.0 / 7.0;
        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = clip<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = wrap<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        pitch_curr = imu->INS_angle[2];
        yaw_curr = imu->INS_angle[0];

        gimbal->TargetRel(pitch_diff, yaw_diff);

        gimbal->UpdateIMU(pitch_curr, yaw_curr);
        osDelay(1);
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart6);

    can1 = new bsp::CAN(&hcan2, false);
    sbus = new remote::SBUS(&huart3);

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

    pitch_motor = new driver::Motor6020(can1, 0x205);
    yaw_motor = new driver::Motor6020(can1, 0x206);
    pitch_motor = new driver::Motor6020(can1, 0x206);
    yaw_motor = new driver::Motor6020(can1, 0x205);

    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(pitch_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t pitch_omega_pid_init = {
        .kp = 200,
        .ki = 1,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                          // 死区
        .A = 1.5 * PI,                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    pitch_motor->ReInitPID(pitch_omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t yaw_omega_pid_init = {
        .kp = 200,
        .ki = 1,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                          // 死区
        .A = 1.5 * PI,                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    yaw_motor->ReInitPID(yaw_omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                       driver::MotorCANBase::ANGLE_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    control::gimbal_t gimbal_data;
    gimbal_data.data = gimbal_init_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;

    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void KillAll() {
    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        pitch_motor->SetOutput(0);
        yaw_motor->SetOutput(0);
        osDelay(10);
    }
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (true) {
        if (sbus->ch5 > 0)
            KillAll();

        set_cursor(0, 0);
        clear_screen();

        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              imu->CaliDone() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", imu->Temp);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
              imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);

        print("\r\n");

        print("CH1: %-4d CH2: %-4d CH3: %-4d CH4: %-4d ", sbus->ch1, sbus->ch2, sbus->ch3,
              sbus->ch4);
        print("CH5: %d CH6: %d CH7: %d CH8: %d @ %d ms\r\n", sbus->ch5, sbus->ch6, sbus->ch7,
              sbus->ch8, sbus->timestamp);

        osDelay(100);
    }
}
