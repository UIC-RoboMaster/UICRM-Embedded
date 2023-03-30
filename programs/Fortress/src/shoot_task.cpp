#include "shoot_task.h"

control::MotorPWMBase* flywheel_left = nullptr;
control::MotorPWMBase* flywheel_right = nullptr;

control::MotorCANBase* steering_motor = nullptr;

control::ServoMotor* load_servo = nullptr;

bsp::GPIO* shoot_key = nullptr;

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
    UNUSED(data);
    float servo_target = servo->GetTarget();
    if (servo_target > servo->GetTheta()) {
        float prev_target = servo->GetTarget() - 2 * PI / 8;
        servo->SetTarget(prev_target, true);
    }
}


osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    // 启动等待
    //TODO: 启动等待

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

    RampSource ramp_1 = RampSource(0, 0, 325, 0.001);
    RampSource ramp_2 = RampSource(0, 0, 325, 0.001);

    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->CalcOutput();

    while (true) {
        //TODO: 离线处理
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

void init_shoot(){
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