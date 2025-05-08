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

#include "AHRS.h"
#include "MPU6500.h"
#include "MotorCanBase.h"
#include "bsp_can.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "heater.h"
#include "main.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

#define RX_SIGNAL (1 << 0)

static control::AHRS* ahrs = nullptr;
static driver::Heater* heater = nullptr;
static bsp::PWM* heater_pwm = nullptr;

static imu::MPU6500* mpu6500 = nullptr;
static bsp::GPIO* imu_cs = nullptr;
static bsp::GPIT* mpu6500_it = nullptr;
static bsp::SPI* spi5 = nullptr;
static bsp::SPIMaster* spi5_master = nullptr;

const osThreadAttr_t imuUpdateTaskAttribute = {.name = "imuUpdateTask",
                                               .attr_bits = osThreadDetached,
                                               .cb_mem = nullptr,
                                               .cb_size = 0,
                                               .stack_mem = nullptr,
                                               .stack_size = 256 * 4,
                                               .priority = (osPriority_t)osPriorityNormal,
                                               .tz_module = 0,
                                               .reserved = 0};

osThreadId_t imuUpdateTaskHandle;

void MPU6500ReceiveDone() {
    osThreadFlagsSet(imuUpdateTaskHandle, RX_SIGNAL);
}

void imuUpdateTask(void* arguments) {
    UNUSED(arguments);
    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            // ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2],
            // mpu6500->accel_[0], mpu6500->accel_[1], mpu6500->accel_[2], mpu6500->mag_[0],
            // mpu6500->mag_[1], mpu6500->mag_[2]);
            ahrs->Update(mpu6500->gyro_[0], mpu6500->gyro_[1], mpu6500->gyro_[2],
                         mpu6500->accel_[0], mpu6500->accel_[1], mpu6500->accel_[2]);
            heater->Update(mpu6500->temperature_);
        }
    }
}

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;

static driver::MotorCANBase* pitch_motor = nullptr;
static driver::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;

const control::gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = 5.1910f,
    .yaw_offset_ = 3.0511f,
    .pitch_max_ = 0.5116f,
    .yaw_max_ = PI,
    .yaw_circle_ = true,
    .pitch_inverted = false,
    .yaw_inverted = true,
    .pitch_eposition = 0,
    .yaw_eposition = 0,
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

    print("Wait for beginning signal...\r\n");
    while (true) {
        if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) {
            break;
        }
        osDelay(100);
    }

    int i = 0;
    while (i < 2000 || mpu6500->temperature_ < 49.0f) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();

        osDelay(1);
        ++i;
    }

    print("Start Calibration.\r\n");
    ahrs->Cailbrate();

    i = 0;
    while (!ahrs->IsCailbrated()) {
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
        pitch_ratio = -dbus->ch3 / 18000.0 / 7.0;
        yaw_ratio = dbus->ch2 / 18000.0 / 7.0;
        pitch_target =
            clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = clip<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = wrap<float>(pitch_target, -PI, PI);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);

        pitch_curr = ahrs->INS_angle[2];
        yaw_curr = ahrs->INS_angle[0];

        gimbal->TargetRel(pitch_diff, yaw_diff);

        gimbal->UpdateIMU(pitch_curr, yaw_curr);

        osDelay(1);
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart8);

    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
    dbus = new remote::DBUS(&huart1);

    imu_cs = new bsp::GPIO(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
    mpu6500_it = new bsp::GPIT(MPU6500_IT_Pin);
    bsp::spi_init_t spi5_init{.hspi = &hspi5, .mode = bsp::SPI_MODE_DMA};
    spi5 = new bsp::SPI(spi5_init);
    bsp::spi_master_init_t spi5_master_init = {
        .spi = spi5,
    };
    spi5_master = new bsp::SPIMaster(spi5_master_init);
    imu::mpu6500_init_t mpu6500_init = {
        .spi = spi5_master,
        .cs = imu_cs,
        .int_pin = mpu6500_it,
        .use_mag = true,
        .dma = true,
    };
    mpu6500 = new imu::MPU6500(mpu6500_init);
    bsp::SetHighresClockTimer(&htim7);

    ahrs = new control::AHRS(false);
    heater_pwm = new bsp::PWM(&htim3, 2, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 50.0f,
    };
    heater = new driver::Heater(heater_init);

    pitch_motor = new driver::Motor6020(can2, 0x20A, 0x2FE);
    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t pitch_motor_theta_pid_init = {
        .kp = 10,
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
    pitch_motor->ReInitPID(pitch_motor_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t pitch_motor_omega_pid_init = {
        .kp = 4000,
        .ki = 100,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 4000,
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
    pitch_motor->ReInitPID(pitch_motor_omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL | driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                         driver::MotorCANBase::ABSOLUTE);
    yaw_motor = new driver::Motor6020(can1, 0x209, 0x2FE);
    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_motor_theta_pid_init = {
        .kp = 8,
        .ki = 0,
        .kd = 1,
        .max_out = 3 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_motor_theta_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t yaw_motor_omega_pid_init = {
        .kp = 2000,
        .ki = 50,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 4000,
        .deadband = 0,                            // 死区
        .A = 0.5 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 0.5 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,      // 输出滤波系数
        .derivative_filtering_coefficient = 0.1,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |             // 积分限幅
                control::ConstrainedPID::OutputFilter |               // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |        // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |       // 变速积分
                control::ConstrainedPID::Derivative_On_Measurement |  // 微分在测量值上
                control::ConstrainedPID::DerivativeFilter             // 微分在测量值上
    };
    yaw_motor->ReInitPID(yaw_motor_omega_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL | driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                       driver::MotorCANBase::ABSOLUTE);

    control::gimbal_t gimbal_data;
    gimbal_data.data = gimbal_init_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}

void RM_RTOS_Threads_Init(void) {
    imuUpdateTaskHandle = osThreadNew(imuUpdateTask, nullptr, &imuUpdateTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void KillAll() {
    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        if (dbus->keyboard.bit.V) {
            break;
        }
        pitch_motor->Disable();
        yaw_motor->Disable();

        osDelay(10);
    }
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    mpu6500->RegisterCallback(MPU6500ReceiveDone);

    while (true) {
        if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN)
            KillAll();

        set_cursor(0, 0);
        clear_screen();

        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              ahrs->IsCailbrated() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", mpu6500->temperature_);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", ahrs->INS_angle[0] / PI * 180,
              ahrs->INS_angle[1] / PI * 180, ahrs->INS_angle[2] / PI * 180);

        print("\r\n");

        print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2,
              dbus->ch3);
        print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);

        osDelay(100);
    }
}
