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

namespace control {

    Chassis::Chassis(const chassis_t chassis) {
        // acquired from user
        model_ = chassis.model;

        // data initialization using acquired model
        switch (chassis.model) {
                // 麦克纳姆轮
            case CHASSIS_MECANUM_WHEEL: {
                // 新建电机关联
                motors_ = new driver::MotorCANBase*[FourWheel::motor_num];
                motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
                motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
                motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
                motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

                // 功率限制系统
                power_limit_ = new PowerLimit(FourWheel::motor_num);

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
        if (power_limit_on_)
            driver::MotorCANBase::RegisterPreOutputCallback(UpdatePowerLimitWrapper, this);
        if (chassis.has_super_capacitor) {
            has_super_capacitor_ = true;
            super_capacitor_ = chassis.super_capacitor;
        }
    }

    Chassis::~Chassis() {
        driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); }, nullptr);
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL: {
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
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                float scale = 1;
                float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
                scale = move_sum > max_motor_speed_ ? max_motor_speed_ / move_sum : 1;

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

    void Chassis::SetPower(bool power_limit_on, float power_limit, float chassis_power,
                           float chassis_power_buffer, bool enable_supercap) {
        if (!power_limit_on_ && power_limit_on) {
            driver::MotorCANBase::RegisterPreOutputCallback(UpdatePowerLimitWrapper, this);
        } else if (power_limit_on_ && !power_limit_on) {
            driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); },
                                                            nullptr);
        }
        power_limit_on_ = power_limit_on;
        power_limit_info_.power_limit = power_limit;
        power_limit_info_.WARNING_power = power_limit * 0.9;
        power_limit_info_.WARNING_power_buff = 50;
        current_chassis_power_ = chassis_power;
        current_chassis_power_buffer_ = chassis_power_buffer;
        if (has_super_capacitor_) {
            if (enable_supercap && !super_capacitor_enable_) {
                super_capacitor_enable_ = true;
            } else if (!enable_supercap && super_capacitor_enable_) {
                super_capacitor_enable_ = false;
            }
            super_capacitor_->SetPowerTotal(chassis_power);
            super_capacitor_->UpdateCurrentBuffer(chassis_power_buffer);
        }
    }

    void Chassis::Update() {
        if (!chassis_enable_) {
            for (int i = 0; i < wheel_num_; i++) {
                motors_[i]->Disable();
            }
            return;
        } else {
            for (int i = 0; i < wheel_num_; i++) {
                if (!motors_[i]->IsEnable())
                    motors_[i]->Enable();
            }
        }

        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
                power_limit_info_.buffer_total_current_limit = 3500 * FourWheel::motor_num;
                power_limit_info_.power_total_current_limit =
                    5000 * FourWheel::motor_num / 80.0 * power_limit_info_.power_limit;
                break;

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }

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
        if (is_enable) {
            if (!power_limit_on_) {
                driver::MotorCANBase::RegisterPreOutputCallback(UpdatePowerLimitWrapper, this);
            }
            power_limit_on_ = true;
        } else {
            if (power_limit_on_) {
                driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); },
                                                                nullptr);
            }
            power_limit_on_ = false;
        }
        bool enable_supercap = data.data_eight_uint8.data[1];
        if (enable_supercap && has_super_capacitor_) {
            super_capacitor_enable_ = true;
        } else {
            super_capacitor_enable_ = false;
        }
        power_limit_info_.power_limit = data.data_two_float.data[1];
        power_limit_info_.WARNING_power = data.data_two_float.data[1] * 0.9f;
        if (has_super_capacitor_) {
            super_capacitor_->SetPowerTotal(power_limit_info_.power_limit);
            if(super_capacitor_->IsOnline()) {
                super_capacitor_->TransmitSettings();
            }
        }
    }
    void Chassis::CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        current_chassis_power_ = data.data_two_float.data[0];
        current_chassis_power_buffer_ = data.data_two_float.data[1];
        if (has_super_capacitor_ && chassis_enable_) {
            if(super_capacitor_->IsOnline()){
                super_capacitor_->TransmitSettings();
                super_capacitor_->UpdateCurrentBuffer(current_chassis_power_buffer_);
            }
        }
    }
    void Chassis::CanBridgeSetTxId(uint8_t tx_id) {
        can_bridge_tx_id_ = tx_id;
    }
    void Chassis::UpdatePowerLimitWrapper(void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->UpdatePowerLimit();
    }
    void Chassis::UpdatePowerLimit() {
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL:{
                float input[FourWheel::motor_num];
                float output[FourWheel::motor_num];
                for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                    input[i] = motors_[i]->GetOutput();
                if (!super_capacitor_enable_) {
                    power_limit_->Output(power_limit_on_, power_limit_info_, current_chassis_power_,
                                         current_chassis_power_buffer_, input, output);
                } else {
                    power_limit_t power_limit_info = power_limit_info_;
                    float supercap_percent = super_capacitor_->GetPercentage();
                    if (supercap_percent < 0.25f) {
                        power_limit_info.power_limit =
                            (0.9f + supercap_percent) * power_limit_info.power_limit;
                        power_limit_info.WARNING_power = power_limit_info.power_limit * 0.9f;
                        power_limit_->Output(power_limit_on_, power_limit_info,
                                             current_chassis_power_, current_chassis_power_buffer_,
                                             input, output);
                    } else {
                        for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                            output[i] = input[i];
                    }
                }

                for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                    motors_[i]->SetOutput((int16_t)output[i]);
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
            if(super_capacitor_->IsOnline()){
                super_capacitor_->TransmitSettings();

            }
        }
    }
    void Chassis::Disable() {
        chassis_enable_ = false;
        if (has_super_capacitor_) {
            super_capacitor_->Disable();
            if(super_capacitor_->IsOnline()){
                super_capacitor_->TransmitSettings();

            }
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
        }
    }
    void ChassisCanBridgeSender::SetPower(bool power_limit_on, float power_limit,
                                          float chassis_power, float chassis_power_buffer,
                                          bool enable_supercap, bool force_update) {
        if (chassis_enable_) {
            if (power_limit_on != chassis_power_limit_on_ || power_limit != chassis_power_limit_ ||
                enable_supercap != chassis_super_capacitor_enable_ || force_update) {
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
