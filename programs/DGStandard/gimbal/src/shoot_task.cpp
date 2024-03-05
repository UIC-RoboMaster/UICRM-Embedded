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

#include "shoot_task.h"

static driver::MotorPWMBase* flywheel_left = nullptr;
static driver::MotorPWMBase* flywheel_right = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

driver::ServoMotor* load_servo = nullptr;

bsp::GPIO* shoot_key = nullptr;

// 堵转回调函数
void jam_callback(driver::ServoMotor* servo, const driver::servo_jam_t data) {
    UNUSED(data);
    // 反向旋转
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
    osDelay(1000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }
    // 等待IMU初始化
    while (ahrs->IsCailbrated()) {
        osDelay(SHOOT_OS_DELAY);
    }

    // 初始化摩擦轮转速斜坡和各状态
    //    int last_state = remote::MID;
    //    int last_state_2 = remote::MID;
    //    uint8_t shoot_state = 0;
    int shoot_flywheel_offset = 0;
    //    uint8_t shoot_state_2 = 0;
    //    uint8_t last_shoot_key = 0;
    uint8_t shoot_state_key = 0;
    uint8_t shoot_state_key_storage = 0;  // 射击状态保存
    //    uint16_t shoot_time_count = 0;
    uint8_t servo_back = 0;
    //    bool can_shoot_click = false;
    RampSource ramp_1 = RampSource(0, 0, 800, 0.001);
    RampSource ramp_2 = RampSource(0, 0, 800, 0.001);

    // 拨弹电机锁原位
    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->CalcOutput();

    ShootMode last_shoot_mode = SHOOT_MODE_STOP;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            // 死了
            shoot_flywheel_offset = -5000;
            ramp_1.Calc(shoot_flywheel_offset);
            ramp_2.Calc(shoot_flywheel_offset);
            //            shoot_state = 0;
            //            shoot_state_2 = 0;
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        //                if (referee->bullet_remaining.bullet_remaining_num_17mm == 0){
        //                    //没子弹了
        //                    shoot_flywheel_offset = -200;
        //                    flywheel_left->SetOutput(ramp_1.Calc(shoot_flywheel_offset));
        //                    flywheel_right->SetOutput(ramp_2.Calc(shoot_flywheel_offset));
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                    kill_shoot();
        //                    osDelay(SHOOT_OS_DELAY);
        //                    continue;
        //                }
        //   检测摩擦轮模式
        switch (shoot_fric_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
            case SHOOT_FRIC_MODE_PREPARED:
                // 摩擦轮加速或者保持满速度
                shoot_flywheel_offset = 400;
                // laser->SetOutput(255);
                break;
            case SHOOT_FRIC_MODE_STOP:
                // 摩擦轮减速或者停止
                shoot_flywheel_offset = -400;
                // laser->SetOutput(0);
                break;
            case SHOOT_FRIC_SPEEDUP:
                // 摩擦轮加速
                ramp_1.SetMax(min(900.0f, ramp_1.GetMax() + 25));
                ramp_2.SetMax(min(900.0f, ramp_2.GetMax() + 25));
                shoot_fric_mode = SHOOT_FRIC_MODE_PREPARING;
                break;
            case SHOOT_FRIC_SPEEDDOWN:
                // 摩擦轮减速
                ramp_1.SetMax(max(500.0f, ramp_1.GetMax() - 25));
                ramp_2.SetMax(max(500.0f, ramp_2.GetMax() - 25));
                shoot_fric_mode = SHOOT_FRIC_MODE_PREPARING;
                break;
            default:
                // 摩擦轮强制停止
                shoot_flywheel_offset = -1000;
                // laser->SetOutput(0);
                break;
        }
        // 计算斜坡函数，摩擦轮输出
        flywheel_left->SetOutput(ramp_1.Calc(shoot_flywheel_offset));
        flywheel_right->SetOutput(ramp_2.Calc(shoot_flywheel_offset));
        if (ramp_1.Get() == ramp_1.GetMax() && ramp_2.Get() == ramp_2.GetMax() &&
            shoot_fric_mode == SHOOT_FRIC_MODE_PREPARING) {
            // 准备就绪判断
            shoot_fric_mode = SHOOT_FRIC_MODE_PREPARED;
        }
        if (shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED) {
            // 如果摩擦轮准备就绪，则检测发射模式
            shoot_state_key = shoot_key->Read();
            switch (shoot_mode) {
                case SHOOT_MODE_PREPARING:
                    // 发射准备就绪检测
                    if (servo_back == 0) {
                        // 第一次发射需要适当退弹
                        load_servo->SetTarget(load_servo->GetTheta() - 2 * PI / 32, true);
                        servo_back = 1;
                    }
                    if (shoot_state_key == 1) {
                        // 检测到准备就绪，转换模式与锁定拔弹电机
                        shoot_mode = SHOOT_MODE_PREPARED;
                        if (!load_servo->Holding()) {
                            load_servo->SetTarget(load_servo->GetTheta(), true);
                        }
                    } else {
                        // 没有准备就绪，则旋转拔弹电机
                        load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
                    }
                    break;
                case SHOOT_MODE_PREPARED:
                    // 准备就绪，未发射状态
                    // 如果检测到未上膛（刚发射一枚子弹），则回到准备模式
                    shoot_state_key_storage = 0;
                    if (shoot_state_key == 0) {
                        shoot_mode = SHOOT_MODE_PREPARING;
                        break;
                    }
                    if (!load_servo->Holding()) {
                        load_servo->SetTarget(load_servo->GetTheta(), true);
                    }
                    break;
                case SHOOT_MODE_SINGLE:
                    // 发射一枚子弹
                    if (shoot_state_key_storage == 0 && shoot_state_key == 0) {
                        shoot_state_key_storage = 1;
                    } else if (shoot_state_key_storage == 1 && shoot_state_key == 1) {
                        shoot_state_key_storage = 0;
                        shoot_mode = SHOOT_MODE_PREPARED;
                        break;
                    }
                    if (last_shoot_mode != SHOOT_MODE_SINGLE)
                        load_servo->SetTarget(load_servo->GetTheta() + 2 * PI / 8, true);
                    else
                        load_servo->SetTarget(load_servo->GetTheta() + 2 * PI / 8, false);
                    break;
                case SHOOT_MODE_BURST:
                    // 连发子弹
                    load_servo->SetTarget(load_servo->GetTarget() + 2 * PI, true);
                    shoot_state_key_storage = 0;
                    break;
                case SHOOT_MODE_STOP:
                    // 停止发射
                    break;
                default:
                    break;
            }
        }
        last_shoot_mode = shoot_mode;

        load_servo->CalcOutput();
        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::MotorPWMBase(&htim1, 1, 1000000, 500, 1000);
    flywheel_right = new driver::MotorPWMBase(&htim1, 4, 1000000, 500, 1000);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);

    steering_motor = new driver::Motor2006(can1, 0x207);

    driver::servo_t servo_data;
    servo_data.motor = steering_motor;
    servo_data.max_speed = 2.5 * PI;
    servo_data.max_acceleration = 16 * PI;
    servo_data.transmission_ratio = M2006P36_RATIO;
    servo_data.omega_pid_param = new float[3]{6000, 80, 0.3};
    servo_data.max_iout = 4000;
    servo_data.max_out = 10000;
    servo_data.hold_pid_param = new float[3]{150, 2, 0.01};
    servo_data.hold_max_iout = 2000;
    servo_data.hold_max_out = 10000;

    load_servo = new driver::ServoMotor(servo_data);
    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->RegisterJamCallback(jam_callback, 0.6);

    shoot_key = new bsp::GPIO(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
}