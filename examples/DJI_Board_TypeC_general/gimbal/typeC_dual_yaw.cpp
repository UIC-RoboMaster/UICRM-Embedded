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

#include "bsp_can.h"
#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dji_dbus.h"
#include "dual_yaw_gimbal.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "utils.h"

static bsp::CAN* can1 = nullptr;
static remote::DBUS* dbus = nullptr;

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute =
    {.name = "imuTask",
     .attr_bits = osThreadDetached,
     .cb_mem = nullptr,
     .cb_size = 0,
     .stack_mem = nullptr,
     .stack_size = 256 * 4,
     .priority = (osPriority_t)osPriorityRealtime,
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

static driver::DjiMotorBase* pitch_motor = nullptr;
static driver::DjiMotorBase* upper_yaw_motor = nullptr;
static driver::DjiMotorBase* lower_yaw_motor = nullptr;
static control::Dual_Yaw_Gimbal* gimbal = nullptr;
static control::dual_yaw_gimbal_data_t* gimbal_param = nullptr;

/* 大小yaw参数：offset / joint_inverted 需要在现场标定（见 programs/Seer/README.md），
   这里给的是占位值，直接上车前务必重新标。 */
const control::dual_yaw_gimbal_data_t gimbal_init_data = {
    /* pitch 轴 */
    .pitch_offset_ = 2.8582f,
    .pitch_max_ = 0.4897f,  // ~28°
    .pitch_inverted = false,
    .pitch_deadband = 0,

    /* 上yaw(小yaw)，角度相对下yaw */
    .upper_yaw_offset_ = -2.1f ,
    .upper_yaw_max_ = PI / 2,  // 关节行程 ±90°
    .upper_yaw_circle_ = false,
    .upper_yaw_inverted = false,  // 遥控 yaw 正方向与云台朝向正方向相反时置 true
    .upper_yaw_deadband = 0,

    /* 下yaw(大yaw)，角度相对车身 */
    .lower_yaw_offset_ = 1.5f,
    .lower_yaw_max_ = PI,
    .lower_yaw_circle_ = true,  // 可连续旋转
    .lower_yaw_inverted = false,
    .lower_yaw_deadband = 0,

    /* 关节角正方向标定：电机反装/编码器反接时置 true */
    .upper_yaw_joint_inverted = false,
    .lower_yaw_joint_inverted = false,

    // 下yaw每周期接手/回中的步长上限 [rad]；0 = 下yaw不主动接手（只补上yaw顶限位的差额）。
    // 0.02 rad ≈ 1.15°，配合约 1kHz 控制周期约 60°/s 的回中速度。
    .lower_yaw_recenter_max_step = 0.2f                                                                                                                                                                                                                                                               ,
};

const osThreadAttr_t gimbalTaskAttribute =
    {.name = "gimbalTask",
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

    // 任务启动时先关掉三个电机，然后等待上电稳定（参考 programs/Ares gimbal_task）
    // pitch_motor->Disable();
    // upper_yaw_motor->Disable();
    // lower_yaw_motor->Disable();
    osDelay(1500);

    // 预热/等待 IMU，先按编码器开环把云台稳在中心。
    // 不要用遥控器拨杆作为开始校准的前置条件：没有接收机时永远不会开始校准，
    // 默认任务就会一直显示 IMU Not Ready。
    int i = 0;
    while (i < 2000 || !imu->DataReady()) {
        gimbal->TargetAbs(0, 0);
        gimbal->UpdateEncoder();
        osDelay(1);
        ++i;
    }

    print("Start Calibration.\r\n");
    imu->Calibrate();

    i = 0;
    while (!imu->DataReady() || !imu->CaliDone()) {
        gimbal->TargetAbs(0, 0);
        gimbal->UpdateEncoder();
        osDelay(1);
        ++i;
    }

    print("Dual Yaw Gimbal Begin!\r\n");

    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
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

        // 每周期增量（相对目标方向），与单yaw example 一致；yaw 目标朝向不受 upper_yaw_max_ 限幅
        pitch_ratio = dbus->ch3 / 18000.0 / 7.0;
        yaw_ratio = dbus->ch2 / 18000.0 / 7.0;
        pitch_diff = clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_diff = clip<float>(yaw_ratio, -PI / 7.0f, PI / 7.0f);

        pitch_curr = imu->INS_angle[2];
        yaw_curr = imu->INS_angle[0];

        gimbal->TargetRel(pitch_diff, yaw_diff);
        gimbal->UpdateIMU(pitch_curr, yaw_curr);

        osDelay(1);
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);

    bsp::SetHighresClockTimer(&BOARD_TIM_SYS);

    can1 = new bsp::CAN(&hcan1, true);
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
    // IMU 必须装在云台(上yaw输出)上，imu_yaw 才是枪口相对地面的朝向
    imu = new IMU(imu_init, false);

    lower_yaw_motor = new driver::Motor6020(can1, 0x205);
    upper_yaw_motor = new driver::Motor6020(can1, 0x209);
    pitch_motor = new driver::Motor6020(can1, 0x208);

    // pitch 和 上yaw(小yaw) 轻、快，可以共用一套；下yaw(大yaw) 重、静摩擦/偏心负载大，
    // 出力要明显更大，必须单独给一套 PID。下面的数值是占位值，上车后要现场标。
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
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
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 200,
        .ki = 1,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                                          // 死区
        .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    // 下yaw(大yaw)专用：位置环增益调大，让速度环更快顶到电流上限；
    // 速度环 kp / 积分上限也加大，保证持续输出足够的力矩。
    control::ConstrainedPID::PID_Init_t lower_yaw_theta_pid_init = {
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
    control::ConstrainedPID::PID_Init_t lower_yaw_omega_pid_init = {
        .kp = 200,
        .ki = 1,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,                                          // 死区
        .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    pitch_motor->SetTransmissionRatio(1);
    pitch_motor->ReInitPID(theta_pid_init, driver::DjiMotorBase::THETA);
    pitch_motor->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    pitch_motor->SetMode(
        driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE
    );

    upper_yaw_motor->SetTransmissionRatio(1);
    upper_yaw_motor->ReInitPID(theta_pid_init, driver::DjiMotorBase::THETA);
    upper_yaw_motor->ReInitPID(omega_pid_init, driver::DjiMotorBase::OMEGA);
    upper_yaw_motor->SetMode(
        driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE
    );

    // 下yaw(大yaw) 用单独一套更强的 PID
    lower_yaw_motor->SetTransmissionRatio(1);
    lower_yaw_motor->ReInitPID(lower_yaw_theta_pid_init, driver::DjiMotorBase::THETA);
    lower_yaw_motor->ReInitPID(lower_yaw_omega_pid_init, driver::DjiMotorBase::OMEGA);
    lower_yaw_motor->SetMode(
        driver::DjiMotorBase::THETA | driver::DjiMotorBase::OMEGA | driver::DjiMotorBase::ABSOLUTE
    );

    control::dual_yaw_gimbal_t gimbal_data;
    gimbal_data.data = gimbal_init_data;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.upper_yaw_motor = upper_yaw_motor;
    gimbal_data.lower_yaw_motor = lower_yaw_motor;

    gimbal = new control::Dual_Yaw_Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void KillAll() {
    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        if (dbus->keyboard.bit.V) {
            break;
        }
        pitch_motor->SetOutput(0);
        upper_yaw_motor->SetOutput(0);
        lower_yaw_motor->SetOutput(0);
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

        print(
            "# %.2f s, IMU %s\r\n",
            HAL_GetTick() / 1000.0,
            imu->CaliDone() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m"
        );
        print("Temp: %.2f\r\n", imu->Temp);
        print(
            "Euler Angles: %.2f, %.2f, %.2f\r\n",
            imu->INS_angle[0] / PI * 180,
            imu->INS_angle[1] / PI * 180,
            imu->INS_angle[2] / PI * 180
        );

        print("\r\n");
        print(
            "Yaw joint: upper %6.1f deg, lower %6.1f deg\r\n",
            gimbal->getUpperYawByMotor(),
            gimbal->getLowerYawByMotor()
        );
        print(
            "Yaw target: upper %6.1f deg, lower %6.1f deg\r\n",
            (gimbal->getUpperYawTarget() - gimbal_param->upper_yaw_offset_) ,
            gimbal->getLowerYawTarget()
        );

        print("\r\n");
        print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
        print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->GetLastUptime());

        osDelay(100);
    }
}
