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

#define M3508_POWER_MODEL {0.000931, 0.000455, 0.000006, 39.151850}

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

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
        chassis_offset_ = chassis.offset;
        power_limit_on_ = chassis.power_limit_on;

        // 功率限制系统
        NewPowerLimit::power_param_t power_model[4] = {
            M3508_POWER_MODEL,
            M3508_POWER_MODEL,
            M3508_POWER_MODEL,
            M3508_POWER_MODEL
        };
        power_limiter_ = new NewPowerLimit(power_model);
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

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetSpeed(const float x_speed, const float y_speed, const float turn_speed) {
        Heartbeat();
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

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetPower(bool enabled, float max_power, float current_power,
                           float buffer_remain, bool enable_supercap) {
        if (power_limit_on_ != enabled)
            power_limit_on_ = enabled;

        UNUSED(max_power);
        UNUSED(current_power);
        UNUSED(buffer_remain);
        UNUSED(enable_supercap);

        return;
    }

    void Chassis::UpdatePower(bool enabled, float max_watt, float current_voltage, float buffer_percent)
    {
        power_limit_on_ = enabled;

        // 电流(A) = 功率(W) / 电压(V)
        float max_current = max_watt / current_voltage;

        // 使用线性插值，调整最大电流。
        if (buffer_percent < 0.5)
        {
            // 如果剩余能量太小，则使用更加严格的限制策略
            // 将0~0.5的buffer映射到0~0.9倍的最大电流
            chassis_power_max_current_ = max_current * linear_interpolation<float>(0.5, 0.8, 0, 0.9, buffer_percent);
        }
        else
        {
            // 正常情况下，使用较为宽松的策略
            // 将0.5~1的buffer映射到0.9~1.2倍的最大电流
            chassis_power_max_current_ = max_current * linear_interpolation<float>(0.5, 0.8, 0.9, 1.2, buffer_percent);
        }

        // 这里仅记录信息，实际在在所有电机的PID计算完成、准备发送CAN前，调用ApplyPowerLimit直接设置电机输出，以进行功率限制
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
        bool is_enable = data.data_eight_uint8.data[0];
        if (is_enable != power_limit_on_)
            power_limit_on_ = is_enable;

        bool enable_supercap = data.data_eight_uint8.data[1];
        if (enable_supercap && has_super_capacitor_) {
            super_capacitor_enable_ = true;
        } else {
            super_capacitor_enable_ = false;
        }
    }
    void Chassis::CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        UNUSED(data);
        //current_chassis_power_ = data.data_two_float.data[0];
        //current_chassis_power_buffer_ = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeSetTxId(uint8_t tx_id) {
        can_bridge_tx_id_ = tx_id;
    }
    void Chassis::ApplyPowerLimitWrapper(void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->ApplyPowerLimit();
    }
    void Chassis::ApplyPowerLimit() {
        if (!power_limit_on_) {
            return;
        }

        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                int16_t turn_current[FourWheel::motor_num];
                float angular_velocity[FourWheel::motor_num];
                for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                {
                    turn_current[i] = motors_[i]->GetOutput();
                    angular_velocity[i] = motors_[i]->GetOmega();
                }

                power_limiter_->LimitPower(turn_current, angular_velocity, (int16_t) (chassis_power_max_current_ * 1000));

                for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                    motors_[i]->SetOutput((int16_t)turn_current[i]);
                break;
            }
            default:
                break;
        }
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

} /* namespace control */
