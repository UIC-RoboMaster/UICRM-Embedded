#include "main.h"

#include "bsp_buzzer.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "gimbal.h"
#include "i2c.h"
#include "spi.h"

static bsp::CAN* can1 = nullptr;
static remote::DBUS* dbus = nullptr;

static bsp::Buzzer* buzzer = nullptr;

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

static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;

static bsp::CAN* can = nullptr;
static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;

static control::Chassis* chassis = nullptr;

static control::MotorPWMBase* flywheel_left = nullptr;
static control::MotorPWMBase* flywheel_right = nullptr;

static control::MotorCANBase* steering_motor = nullptr;

static control::ServoMotor* load_servo = nullptr;

static bsp::GPIO* shoot_key = nullptr;

uint32_t last_timestamp = 0;

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

    control::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor, steering_motor};
    osDelay(2000);
    while (true) {
        if ((HAL_GetTick() - last_timestamp) < 500 &&
            (dbus->keyboard.bit.V || dbus->swr != remote::DOWN)) {
            break;
        }
        osDelay(10);
    }
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
        if (HAL_GetTick() - last_timestamp > 550) {
            while (1) {
                if (HAL_GetTick() - last_timestamp < 500)
                    break;
                osDelay(10);
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
        //    if (dbus->swr == remote::UP) {
        //      gimbal->TargetAbs(0, 0);
        //      gimbal->Update();
        //      pitch_target = pitch_curr;
        //      yaw_target = yaw_curr;
        //      control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        //      osDelay(1);
        //      continue;
        //    }
        pitch_ratio = dbus->mouse.y / 32767.0 * 7.5 / 7.0;
        yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5 / 7.0;
        pitch_ratio = dbus->ch3 / 18000.0 / 7.0;
        yaw_ratio = dbus->ch4 / 18000.0 / 7.0;
        pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                                   gimbal_param->pitch_max_);
        yaw_target =
            wrap<float>(yaw_target + yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
        yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

        if (-0.005 < pitch_diff && pitch_diff < 0.005) {
            pitch_diff = 0;
        }
        if (dbus->swr == remote::UP) {
            gimbal->TargetAbsYawRelPitch(pitch_diff, 0);
            gimbal->Update();
            yaw_target = yaw_curr;
        } else {
            gimbal->TargetRel(pitch_diff, yaw_diff);
        }

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
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t chassisTaskHandle;

void chassisTask(void* arg) {
    UNUSED(arg);

    control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

    while (true) {
        if ((HAL_GetTick() - last_timestamp) < 500 &&
            (dbus->keyboard.bit.V || dbus->swr != remote::DOWN)) {
            break;
        }
        osDelay(10);
    }
    while (!imu->DataReady() || !imu->CaliDone()) {
        osDelay(1);
    }

    float relative_angle = yaw_motor->GetAngleOffset(gimbal_param->yaw_offset_);

    // float last_speed = 0;
    float sin_yaw, cos_yaw, vx_set, vy_set, vz_set, vx_set_org, vy_set_org;
    while (true) {
        if (HAL_GetTick() - last_timestamp > 550) {
            while (true) {
                if (HAL_GetTick() - last_timestamp < 500)
                    break;
                osDelay(10);
            }
            continue;
        }
        if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) {
            while (true) {
                if (dbus->keyboard.bit.V || dbus->swr != remote::DOWN) {
                    break;
                }
                osDelay(10);
            }
            continue;
        }
        if (dbus->swr == remote::UP) {
            chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
        } else {
            relative_angle = yaw_motor->GetAngleOffset(gimbal_param->yaw_offset_);

            sin_yaw = arm_sin_f32(relative_angle);
            cos_yaw = arm_cos_f32(relative_angle);
            vx_set_org = dbus->ch0;
            vy_set_org = dbus->ch1;
            vx_set = cos_yaw * vx_set_org + sin_yaw * vy_set_org;
            vy_set = -sin_yaw * vx_set_org + cos_yaw * vy_set_org;
            vz_set = dbus->ch2;
            chassis->SetSpeed(vx_set, vy_set, vz_set);
        }

        chassis->Update(false, 30, 20, 60);
        control::MotorCANBase::TransmitOutput(motors, 4);
        osDelay(1);
    }
}

#define SHOOT_OS_DELAY 1

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
    UNUSED(data);
    float servo_target = servo->GetTarget();
    if (servo_target > servo->GetTheta()) {
        float prev_target = servo->GetTarget() - 2 * PI / 8;
        servo->SetTarget(prev_target, true);
    }
}

const osThreadAttr_t shootTaskAttribute = {.name = "shootTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 256 * 4,
                                           .priority = (osPriority_t)osPriorityNormal,
                                           .tz_module = 0,
                                           .reserved = 0};
osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    // 启动等待
    while (true) {
        if ((HAL_GetTick() - last_timestamp) < 500 &&
            (dbus->keyboard.bit.V || dbus->swr != remote::DOWN)) {
            break;
        }
        osDelay(10);
    }
    // 等待IMU初始化
    while (!imu->DataReady() || !imu->CaliDone()) {
        osDelay(1);
    }
    int last_state = remote::MID;
    int last_state_2 = remote::MID;
    uint8_t shoot_state = 0;
    int shoot_flywheel_offset = 0;
    uint8_t shoot_state_2 = 0;
    uint8_t last_shoot_key = 0;
    uint8_t shoot_state_key = 0;
    uint16_t shoot_time_count = 0;
    uint8_t servo_back = 0;

    RampSource ramp_1 = RampSource(0, 0, 450, 0.001);
    RampSource ramp_2 = RampSource(0, 0, 450, 0.001);

    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->CalcOutput();

    while (true) {
        if (HAL_GetTick() - last_timestamp > 550) {
            while (true) {
                if (HAL_GetTick() - last_timestamp < 500)
                    break;
                osDelay(10);
            }
            continue;
        }
        if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) {
            while (true) {
                if (dbus->keyboard.bit.V || dbus->swr != remote::DOWN) {
                    break;
                }
                osDelay(10);
            }
            continue;
        }
        // 检测开关状态，向上来回打即启动拔弹
        if (dbus->swl == remote::UP) {
            if (last_state == remote::MID)
                last_state = remote::UP;
        } else if (dbus->swl == remote::MID) {
            if (last_state == remote::UP) {
                last_state = remote::MID;
                if (shoot_state == 0) {
                    shoot_state = 1;
                } else {
                    shoot_state = 0;
                    shoot_state_2 = 0;
                }
            }
        }
        switch (shoot_state) {
            case 0:
                shoot_flywheel_offset = -200;

                break;
            case 1:
            case 2:
                shoot_flywheel_offset = 200;
                if (servo_back == 0) {
                    load_servo->SetTarget(load_servo->GetTheta() - 2 * PI / 32, true);
                    servo_back = 1;
                }
                break;
        }
        if (shoot_state == 1 && ramp_1.Get() == ramp_1.GetMax() &&
            ramp_2.Get() == ramp_2.GetMax()) {
            shoot_state = 2;
        }
        flywheel_left->SetOutput(ramp_1.Calc(shoot_flywheel_offset));
        flywheel_right->SetOutput(ramp_2.Calc(shoot_flywheel_offset));
        // 启动拔弹电机后的操作
        if (shoot_state == 2) {
            // 检测是否已装填子弹
            shoot_state_key = shoot_key->Read();
            // 检测是否需要发射子弹
            if (dbus->swl == remote::DOWN) {
                if (last_state_2 == remote::MID) {
                    last_state_2 = remote::DOWN;
                    if (shoot_state_2 == 0) {
                        shoot_state_2 = 1;
                    }
                    shoot_time_count = 0;
                }
                shoot_time_count++;
                if (shoot_time_count > 1000 / SHOOT_OS_DELAY) {
                    shoot_state_2 = 2;
                }
            } else if (dbus->swl == remote::MID) {
                if (last_state_2 == remote::DOWN) {
                    last_state_2 = remote::MID;
                }
                shoot_state_2 = 0;
            }
            // 发射子弹
            if (shoot_state_2 == 1) {
                // 检测是否已经发射完毕
                if (last_shoot_key == 0 && shoot_state_key == 1) {
                    last_shoot_key = 1;
                } else if (last_shoot_key == 1 && shoot_state_key == 0) {
                    last_shoot_key = 0;
                    shoot_state_2 = 0;
                }
                // 如果发射未完成，则需要发射子弹
                if (shoot_state_2 == 1) {
                    load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
                } else {
                    if (!load_servo->Holding()) {
                        load_servo->SetTarget(load_servo->GetTheta(), true);
                    }
                }
            } else if (shoot_state_2 == 2) {
                // 连续发射
                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
            } else if (shoot_state_key == 1) {
                // 不需要发射子弹，但是未装弹完毕，则需要装填子弹
                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
            } else {
                // 不需要发射子弹，且装弹完毕，则需要锁定拔弹电机
                if (!load_servo->Holding()) {
                    load_servo->SetTarget(load_servo->GetTheta(), true);
                }
            }
        }
        // 计算输出，由于拔弹电机的输出系统由云台托管，不需要再次处理can的传输
        load_servo->CalcOutput();

        osDelay(SHOOT_OS_DELAY);
    }
}

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
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

    control::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);

    flywheel_left = new control::MotorPWMBase(&htim1, 1, 1000000, 500, 1080);
    flywheel_right = new control::MotorPWMBase(&htim1, 2, 1000000, 500, 1080);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);

    control::servo_t servo_data;
    servo_data.motor = steering_motor;
    servo_data.max_speed = 2 * PI;
    servo_data.max_acceleration = 8 * PI;
    servo_data.transmission_ratio = M2006P36_RATIO;
    servo_data.omega_pid_param = new float[3]{150, 2, 0.01};
    servo_data.max_iout = 2000;
    servo_data.max_out = 10000;

    load_servo = new control::ServoMotor(servo_data);
    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->RegisterJamCallback(jam_callback, 0.6);

    shoot_key = new bsp::GPIO(BUTTON_TRI_GPIO_Port, BUTTON_TRI_Pin);
}

void KillAll() {
    control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
    control::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor, steering_motor};

    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        last_timestamp = dbus->timestamp;
        if ((HAL_GetTick() - last_timestamp) < 500 &&
            (dbus->keyboard.bit.V || dbus->swr != remote::DOWN)) {
            break;
        }

        fl_motor->SetOutput(0);
        fr_motor->SetOutput(0);
        bl_motor->SetOutput(0);
        br_motor->SetOutput(0);
        control::MotorCANBase::TransmitOutput(motors, 4);

        pitch_motor->SetOutput(0);
        yaw_motor->SetOutput(0);
        steering_motor->SetOutput(0);
        control::MotorCANBase::TransmitOutput(gimbal_motors, 3);
        flywheel_left->SetOutput(0);
        flywheel_right->SetOutput(0);
        osDelay(10);
    }
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(1000);
    while (true) {
        last_timestamp = dbus->timestamp;
        if (HAL_GetTick() - last_timestamp > 550)
            KillAll();
        if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN)
            KillAll();

        osDelay(200);
    }
}
