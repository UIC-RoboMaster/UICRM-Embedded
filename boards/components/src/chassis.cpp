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

#include "chassis.h"

#include "bsp_error_handler.h"
#include "bsp_os.h"

#define M3508_POWER_MODEL \
    { 0.000931, 0.000455, 0.000006, 39.151850 }

namespace control {

    Chassis::Chassis(const chassis_t chassis) {
        // acquired from user
        model_ = chassis.model;

        // data initialization using acquired model
        switch (chassis.model) {
                // 麦克纳姆轮
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                // 新建电机关联
                motors_ = new driver::MotorCANBase*[FourWheel::motor_num];
                motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
                motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
                motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
                motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

                // 速度初始化
                speeds_ = new float[FourWheel::motor_num];
                for (int i = 0; i < FourWheel::motor_num; ++i)
                    speeds_[i] = 0;

                // 轮子数
                wheel_num_ = FourWheel::motor_num;
                break;
            }

                // 舵轮底盘
            case CHASSIS_STEER_WHEEL: {
                // 新建电机关联 - 8个电机（4个驱动 + 4个转向）
                motors_ = new driver::MotorCANBase*[SteerWheel::motor_num];
                for (int i = 0; i < SteerWheel::motor_num; ++i) {
                    motors_[i] = chassis.motors[i];
                }

                // 速度初始化 - 驱动电机4个 + 转向电机角度4个
                speeds_ = new float[SteerWheel::motor_num];
                for (int i = 0; i < SteerWheel::motor_num; ++i)
                    speeds_[i] = 0;

                // 轮子数
                wheel_num_ = SteerWheel::motor_num;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
        chassis_offset_ = chassis.offset;

        // 功率限制系统
        NewPowerLimit::power_param_t power_model[4] = {M3508_POWER_MODEL, M3508_POWER_MODEL,
                                                       M3508_POWER_MODEL, M3508_POWER_MODEL};
        power_limit_.enabled = chassis.power_limit_on;
        power_limit_.limiter = new NewPowerLimit(power_model);
        driver::MotorCANBase::RegisterPreOutputCallback(ApplyPowerLimitWrapper, this);

        // 底盘是否有超级电容
        if (chassis.has_super_capacitor) {
            has_super_capacitor_ = true;
            super_capacitor_ = chassis.super_capacitor;
        }
    }

    Chassis::~Chassis() {
        driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); }, nullptr);
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                motors_[FourWheel::front_left] = nullptr;
                motors_[FourWheel::front_right] = nullptr;
                motors_[FourWheel::back_left] = nullptr;
                motors_[FourWheel::back_right] = nullptr;
                delete[] motors_;
                motors_ = nullptr;

                delete[] speeds_;
                speeds_ = nullptr;
                break;
            }

            case CHASSIS_STEER_WHEEL: {
                for (int i = 0; i < SteerWheel::motor_num; ++i) {
                    motors_[i] = nullptr;
                }
                delete[] motors_;
                motors_ = nullptr;

                delete[] speeds_;
                speeds_ = nullptr;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetSpeed(const float x_speed, const float y_speed, const float turn_speed) {
        Heartbeat();
        chassis_vx = x_speed;
        chassis_vy = y_speed;
        chassis_vt = turn_speed;

        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
                float scale = move_sum > max_motor_speed_ ? max_motor_speed_ / move_sum : 1.0f;

                speeds_[FourWheel::front_left] =
                    scale * (y_speed + x_speed + turn_speed * (1 - chassis_offset_));  // 2
                speeds_[FourWheel::back_left] =
                    scale * (y_speed - x_speed + turn_speed * (1 + chassis_offset_));  // 3
                speeds_[FourWheel::front_right] =
                    -scale * (y_speed - x_speed - turn_speed * (1 - chassis_offset_));  // 1
                speeds_[FourWheel::back_right] =
                    -scale * (y_speed + x_speed - turn_speed * (1 + chassis_offset_));  // 4
                break;
            }

            case CHASSIS_STEER_WHEEL: {
                // 舵轮底盘运动学解算 (45°对角线布局)
                // 参考: https://github.com/scutrobotlab/RM2010_AGVinfantry
                //
                // 四个轮子相对于底盘中心呈45°对角线分布:
                //        1(前左)          4(前右)
                //              \        /
                //               \  w  /
                //                \  /
                //                 中心
                //                /  \
                //               /    \
                //        2(后左)          3(后右)
                //
                // R: 底盘中心到各轮子的距离 (对角线半径)
                // 需要根据实际机器人尺寸设置，单位：米
                const float R = 0.4f;  // 建议范围: 0.3-0.5米，根据实际调整

                // 45度角的三角函数值
                const float cos45 = 0.707106781f;  // cos(45°) = √2/2
                const float sin45 = 0.707106781f;  // sin(45°) = √2/2

                // 各轮组的平移速度分量 (Vx1-4, Vy1-4)
                // 对于45°布局，所有轮子的Vx和Vy都等于底盘整体速度
                float Vx1 = x_speed, Vy1 = y_speed;
                float Vx2 = x_speed, Vy2 = y_speed;
                float Vx3 = x_speed, Vy3 = y_speed;
                float Vx4 = x_speed, Vy4 = y_speed;

                // 各轮组的旋转速度分量 (Vw1-4)
                // Vw = turn_speed × R
                float Vw1 = turn_speed * R;
                float Vw2 = turn_speed * R;
                float Vw3 = turn_speed * R;
                float Vw4 = turn_speed * R;

                // 轮1 (前左): 位于第二象限，角度135°
                // V1 = √[(Vy1 - Vw1*cos45°)² + (Vx1 - Vw1*sin45°)²]
                // θ1 = atan2((Vy1 - Vw1*cos45°), (Vx1 - Vw1*sin45°))
                float vy1_comp = Vy1 - Vw1 * cos45;
                float vx1_comp = Vx1 - Vw1 * sin45;
                float speed_1 = sqrt(vy1_comp * vy1_comp + vx1_comp * vx1_comp);
                float angle_1 = atan2(vy1_comp, vx1_comp);

                // 轮2 (后左): 位于第三象限，角度225°
                // V2 = √[(Vy2 - Vw2*cos45°)² + (Vx2 + Vw2*sin45°)²]
                // θ2 = atan2((Vy2 - Vw2*cos45°), (Vx2 + Vw2*sin45°))
                float vy2_comp = Vy2 - Vw2 * cos45;
                float vx2_comp = Vx2 + Vw2 * sin45;
                float speed_2 = sqrt(vy2_comp * vy2_comp + vx2_comp * vx2_comp);
                float angle_2 = atan2(vy2_comp, vx2_comp);

                // 轮3 (后右): 位于第四象限，角度315°
                // V3 = √[(Vy3 + Vw3*cos45°)² + (Vx3 + Vw3*sin45°)²]
                // θ3 = atan2((Vy3 + Vw3*cos45°), (Vx3 + Vw3*sin45°))
                float vy3_comp = Vy3 + Vw3 * cos45;
                float vx3_comp = Vx3 + Vw3 * sin45;
                float speed_3 = sqrt(vy3_comp * vy3_comp + vx3_comp * vx3_comp);
                float angle_3 = atan2(vy3_comp, vx3_comp);

                // 轮4 (前右): 位于第一象限，角度45°
                // V4 = √[(Vy4 + Vw4*cos45°)² + (Vx4 - Vw4*sin45°)²]
                // θ4 = atan2((Vy4 + Vw4*cos45°), (Vx4 - Vw4*sin45°))
                float vy4_comp = Vy4 + Vw4 * cos45;
                float vx4_comp = Vx4 - Vw4 * sin45;
                float speed_4 = sqrt(vy4_comp * vy4_comp + vx4_comp * vx4_comp);
                float angle_4 = atan2(vy4_comp, vx4_comp);

                // 找到最大速度用于归一化
                float max_speed = speed_1;
                if (speed_2 > max_speed)
                    max_speed = speed_2;
                if (speed_3 > max_speed)
                    max_speed = speed_3;
                if (speed_4 > max_speed)
                    max_speed = speed_4;

                // 如果最大速度超过限制，进行缩放
                float scale = 1.0f;
                if (max_speed > max_motor_speed_) {
                    scale = max_motor_speed_ / max_speed;
                }

                // 设置驱动电机速度 (索引 0-3)
                // 注意：电机索引对应: 0=前左, 1=前右, 2=后左, 3=后右
                speeds_[SteerWheel::drive_front_left] = speed_1 * scale;   // 轮1
                speeds_[SteerWheel::drive_front_right] = speed_4 * scale;  // 轮4
                speeds_[SteerWheel::drive_back_left] = speed_2 * scale;    // 轮2
                speeds_[SteerWheel::drive_back_right] = speed_3 * scale;   // 轮3

                // 设置转向电机角度 (索引 4-7)
                speeds_[SteerWheel::steer_front_left] = angle_1;   // 轮1
                speeds_[SteerWheel::steer_front_right] = angle_4;  // 轮4
                speeds_[SteerWheel::steer_back_left] = angle_2;    // 轮2
                speeds_[SteerWheel::steer_back_right] = angle_3;   // 轮3

                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetPower(bool enabled, float max_power, float current_power, float buffer_remain,
                           bool enable_supercap) {
        power_limit_.enabled = enabled;

        UNUSED(max_power);
        UNUSED(current_power);
        UNUSED(buffer_remain);
        UNUSED(enable_supercap);

        return;
    }

    void Chassis::UpdatePower(bool enabled, float max_watt, float current_voltage,
                              uint8_t buffer_percent) {
        power_limit_.enabled = enabled;
        power_limit_.buffer_percent = buffer_percent;
        // 双板通信下，云台可以选择不更新电压值，底盘可以使用自己采样的电压值
        if (current_voltage > 1)
            power_limit_.voltage = current_voltage;
        power_limit_.max_watt = max_watt;
        // 这里仅记录信息，实际在在所有电机的PID计算完成、准备发送CAN前，调用ApplyPowerLimit直接设置电机输出，以进行功率限制
    }

    void Chassis::UpdatePowerVoltage(float voltage) {
        power_limit_.voltage = voltage;
    }

    void Chassis::Update() {
        // 如果有电机掉线，则关闭整个底盘
        bool need_shutdown = !IsOnline();
        for (int i = 0; i < wheel_num_; i++) {
            if (!motors_[i]->IsOnline()) {
                need_shutdown = true;
                break;
            }
        }
        if (need_shutdown) {
            Disable();
        }

        // 根据底盘的开关，控制四个电机开关
        if (!chassis_enable_) {
            for (int i = 0; i < wheel_num_; i++) {
                motors_[i]->Disable();
            }
            return;
        }
        for (int i = 0; i < wheel_num_; i++) {
            if (!motors_[i]->IsEnable())
                motors_[i]->Enable();
        }

        // 将解算得到的数据（每个电机的转速）传递给电机类，由电机类进行PID控制、CAN输出等
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                motors_[FourWheel::front_left]->SetTarget(speeds_[FourWheel::front_left]);
                motors_[FourWheel::front_right]->SetTarget(speeds_[FourWheel::front_right]);
                motors_[FourWheel::back_left]->SetTarget(speeds_[FourWheel::back_left]);
                motors_[FourWheel::back_right]->SetTarget(speeds_[FourWheel::back_right]);
                break;
            }

            case CHASSIS_STEER_WHEEL: {
                // 驱动电机设置速度 (索引 0-3，使用速度控制模式 OMEGA)
                motors_[SteerWheel::drive_front_left]->SetTarget(
                    speeds_[SteerWheel::drive_front_left]);
                motors_[SteerWheel::drive_front_right]->SetTarget(
                    speeds_[SteerWheel::drive_front_right]);
                motors_[SteerWheel::drive_back_left]->SetTarget(speeds_[SteerWheel::drive_back_left]);
                motors_[SteerWheel::drive_back_right]->SetTarget(
                    speeds_[SteerWheel::drive_back_right]);

                // 转向电机设置角度 (索引 4-7，使用角度控制模式 THETA + ABSOLUTE)
                motors_[SteerWheel::steer_front_left]->SetTarget(
                    speeds_[SteerWheel::steer_front_left]);
                motors_[SteerWheel::steer_front_right]->SetTarget(
                    speeds_[SteerWheel::steer_front_right]);
                motors_[SteerWheel::steer_back_left]->SetTarget(
                    speeds_[SteerWheel::steer_back_left]);
                motors_[SteerWheel::steer_back_right]->SetTarget(
                    speeds_[SteerWheel::steer_back_right]);
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::CanBridgeUpdateEventXYWrapper(communication::can_bridge_ext_id_t ext_id,
                                                communication::can_bridge_data_t data, void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventXY(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventTurnWrapper(communication::can_bridge_ext_id_t ext_id,
                                                  communication::can_bridge_data_t data,
                                                  void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventTurn(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventPowerLimitWrapper(communication::can_bridge_ext_id_t ext_id,
                                                        communication::can_bridge_data_t data,
                                                        void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventPowerLimit(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventCurrentPowerWrapper(communication::can_bridge_ext_id_t ext_id,
                                                          communication::can_bridge_data_t data,
                                                          void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventCurrentPower(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventXY(communication::can_bridge_ext_id_t ext_id,
                                         communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        can_bridge_vx_ = data.data_two_float.data[0];
        can_bridge_vy_ = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeUpdateEventTurn(communication::can_bridge_ext_id_t ext_id,
                                           communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        can_bridge_vt_ = data.data_two_float.data[1];
        float is_enable = data.data_two_float.data[0];
        if (is_enable > 0.1f && !chassis_enable_) {
            Enable();
        } else if (is_enable < 0.1f && chassis_enable_) {
            Disable();
        }
        if (chassis_enable_) {
            SetSpeed(can_bridge_vx_, can_bridge_vy_, can_bridge_vt_);
        } else {
            SetSpeed(0, 0, 0);
        }
    }
    void Chassis::CanBridgeUpdateEventPowerLimit(communication::can_bridge_ext_id_t ext_id,
                                                 communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }

        bool enabled = data.data_eight_uint8.data[0];
        uint8_t max_watt = data.data_eight_uint8.data[1];
        uint8_t buffer_percent = data.data_eight_uint8.data[2];
        float voltage = data.data_two_float.data[1];
        UpdatePower(enabled, max_watt, voltage, buffer_percent);
    }
    void Chassis::CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        UNUSED(data);
        // current_chassis_power_ = data.data_two_float.data[0];
        // current_chassis_power_buffer_ = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeSetTxId(uint8_t tx_id) {
        can_bridge_tx_id_ = tx_id;
    }
    void Chassis::ApplyPowerLimitWrapper(void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->ApplyPowerLimit();
    }
    void Chassis::ApplyPowerLimit() {
        if (!power_limit_.enabled) {
            return;
        }

        // 电流(A) = 功率(W) / 电压(V)
        float max_current = power_limit_.max_watt / power_limit_.voltage;

        // 根据缓冲区剩余能量，使用线性插值，计算最大电流。
        max_current = max_current *
                      linear_interpolation<int>(20, 80, 80, 150, power_limit_.buffer_percent) / 100;

        // 获取数据
        int16_t turn_current[FourWheel::motor_num];
        float angular_velocity[FourWheel::motor_num];
        for (uint8_t i = 0; i < FourWheel::motor_num; ++i) {
            turn_current[i] = motors_[i]->GetOutput();
            angular_velocity[i] = motors_[i]->GetOmega();
        }

        // 在转矩电流之上应用功率限制
        power_limit_.limiter->LimitPower(turn_current, angular_velocity,
                                         (int16_t)(max_current * 1000));

        // 应用限制后的转矩电流
        for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
            motors_[i]->SetOutput((int16_t)turn_current[i]);
    }
    void Chassis::Enable() {
        chassis_enable_ = true;
        if (has_super_capacitor_) {
            super_capacitor_->Enable();
        }
    }
    void Chassis::Disable() {
        chassis_enable_ = false;
        if (has_super_capacitor_) {
            super_capacitor_->Disable();
        }
    }

    void Chassis::SetMaxMotorSpeed(float max_speed) {
        max_motor_speed_ = max_speed;
    }

    ChassisCanBridgeSender::ChassisCanBridgeSender(communication::CanBridge* can_bridge,
                                                   uint8_t rx_id)
        : can_bridge_(can_bridge), device_rx_id_(rx_id) {
        rx_id_.data.rx_id = rx_id;
        rx_id_.data.type = communication::CAN_BRIDGE_TYPE_TWO_FLOAT;
    }
    void ChassisCanBridgeSender::SetChassisRegId(uint8_t xy_reg_id, uint8_t turn_on_reg_id,
                                                 uint8_t power_limit_reg_id,
                                                 uint8_t current_power_reg_id) {
        chassis_xy_reg_id_ = xy_reg_id;
        chassis_turn_on_reg_id_ = turn_on_reg_id;
        chassis_power_limit_reg_id_ = power_limit_reg_id;
        chassis_current_power_reg_id_ = current_power_reg_id;
    }
    void ChassisCanBridgeSender::Enable() {
        if (!chassis_enable_) {
            chassis_enable_ = true;
            data_.data_two_float.data[0] = 1.0f;
            data_.data_two_float.data[1] = 0.0f;
            rx_id_.data.reg = chassis_turn_on_reg_id_;
            can_bridge_->Send(rx_id_, data_);
        }
    }
    void ChassisCanBridgeSender::Disable() {
        if (chassis_enable_) {
            chassis_enable_ = false;
        }
        data_.data_two_float.data[0] = 0;
        data_.data_two_float.data[1] = 0;
        rx_id_.data.reg = chassis_turn_on_reg_id_;
        can_bridge_->Send(rx_id_, data_);
    }
    void ChassisCanBridgeSender::SetSpeed(const float x_speed, const float y_speed,
                                          const float turn_speed) {
        if (chassis_enable_) {
            data_.data_two_float.data[0] = x_speed;
            data_.data_two_float.data[1] = y_speed;
            rx_id_.data.reg = chassis_xy_reg_id_;
            can_bridge_->Send(rx_id_, data_);
            data_.data_two_float.data[0] = 1.0f;
            data_.data_two_float.data[1] = turn_speed;
            rx_id_.data.reg = chassis_turn_on_reg_id_;
            can_bridge_->Send(rx_id_, data_);
        } else {
            data_.data_two_float.data[0] = 0.0f;
            data_.data_two_float.data[1] = 0.0f;
            can_bridge_->Send(rx_id_, data_);
        }
        chassis_vx = x_speed;
        chassis_vy = y_speed;
        chassis_vt = turn_speed;
    }
    void ChassisCanBridgeSender::SetPower(bool power_limit_on, float power_limit,
                                          float chassis_power, float chassis_power_buffer,
                                          bool enable_supercap, bool force_update) {
        UNUSED(force_update);
        if (chassis_enable_) {
            {
                chassis_power_limit_on_ = power_limit_on;
                chassis_power_limit_ = power_limit;
                data_.data_eight_uint8.data[0] = power_limit_on;
                data_.data_eight_uint8.data[1] = enable_supercap;
                data_.data_two_float.data[1] = power_limit;
                rx_id_.data.reg = chassis_power_limit_reg_id_;
                can_bridge_->Send(rx_id_, data_);
            }
            {
                data_.data_two_float.data[0] = chassis_power;
                data_.data_two_float.data[1] = chassis_power_buffer;
                rx_id_.data.reg = chassis_current_power_reg_id_;
                can_bridge_->Send(rx_id_, data_);
            }
        }
    }
    void ChassisCanBridgeSender::UpdatePower(bool enabled, uint8_t max_watt, float current_voltage,
                                             uint8_t buffer_percent) {
        if (!chassis_enable_) {
            return;
        }
        data_.data_eight_uint8.data[0] = enabled;
        data_.data_eight_uint8.data[1] = max_watt;
        data_.data_eight_uint8.data[2] = buffer_percent;
        data_.data_two_float.data[1] = current_voltage;

        rx_id_.data.reg = chassis_power_limit_reg_id_;
        can_bridge_->Send(rx_id_, data_);
    }

} /* namespace control */
