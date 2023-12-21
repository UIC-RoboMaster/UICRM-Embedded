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

#include "chassis.h"

#include "bsp_error_handler.h"

namespace control {

    Chassis::Chassis(const chassis_t chassis) : pids_() {
        // acquired from user
        model_ = chassis.model;

        // data initialization using acquired model
        switch (chassis.model) {
            case CHASSIS_MECANUM_WHEEL: {
                motors_ = new driver::MotorCANBase*[FourWheel::motor_num];
                motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
                motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
                motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
                motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

                float* pid_param = new float[3]{40, 3, 0};
                float motor_max_iout = 2000;
                float motor_max_out = 30000;
                pids_[FourWheel::front_left].Reinit(pid_param, motor_max_iout, motor_max_out);
                pids_[FourWheel::front_right].Reinit(pid_param, motor_max_iout, motor_max_out);
                pids_[FourWheel::back_left].Reinit(pid_param, motor_max_iout, motor_max_out);
                pids_[FourWheel::back_right].Reinit(pid_param, motor_max_iout, motor_max_out);

                power_limit_ = new PowerLimit(FourWheel::motor_num);

                speeds_ = new float[FourWheel::motor_num];
                for (int i = 0; i < FourWheel::motor_num; ++i)
                    speeds_[i] = 0;

                wheel_num_ = FourWheel::motor_num;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
        chassis_offset_ = chassis.offset;
    }

    Chassis::~Chassis() {
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
            case CHASSIS_MECANUM_WHEEL: {
                float scale = 1;

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
                           float chassis_power_buffer) {
        power_limit_on_ = power_limit_on;
        power_limit_info_.power_limit = power_limit;
        power_limit_info_.WARNING_power = power_limit * 0.9;
        power_limit_info_.WARNING_power_buff = 50;
        current_chassis_power_ = chassis_power;
        current_chassis_power_buffer_ = chassis_power_buffer;
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
                power_limit_info_.buffer_total_current_limit = 3500 * FourWheel::motor_num;
                power_limit_info_.power_total_current_limit =
                    5000 * FourWheel::motor_num / 80.0 * power_limit;
                break;

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }

    }

    void Chassis::Update() {
        if(!chassis_enable_){
            for(int i=0;i<wheel_num_;i++){
                motors_[i]->SetOutput(0);
                pids_[i].Reset();
            }
        }
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL: {

                float PID_output[FourWheel::motor_num];
                float output[FourWheel::motor_num];

                PID_output[FourWheel::front_left] = pids_[FourWheel::front_left].ComputeOutput(
                    motors_[FourWheel::front_left]->GetOmegaDelta(speeds_[FourWheel::front_left]));
                PID_output[FourWheel::back_left] = pids_[FourWheel::back_left].ComputeOutput(
                    motors_[FourWheel::back_left]->GetOmegaDelta(speeds_[FourWheel::back_left]));
                PID_output[FourWheel::front_right] = pids_[FourWheel::front_right].ComputeOutput(
                    motors_[FourWheel::front_right]->GetOmegaDelta(
                        speeds_[FourWheel::front_right]));
                PID_output[FourWheel::back_right] = pids_[FourWheel::back_right].ComputeOutput(
                    motors_[FourWheel::back_right]->GetOmegaDelta(speeds_[FourWheel::back_right]));

                power_limit_->Output(power_limit_on_, power_limit_info_, current_chassis_power_,
                                     current_chassis_power_buffer_, PID_output, output);

                motors_[FourWheel::front_left]->SetOutput(
                    control::ClipMotorRange(output[FourWheel::front_left]));
                motors_[FourWheel::back_left]->SetOutput(
                    control::ClipMotorRange(output[FourWheel::back_left]));
                motors_[FourWheel::front_right]->SetOutput(
                    control::ClipMotorRange(output[FourWheel::front_right]));
                motors_[FourWheel::back_right]->SetOutput(
                    control::ClipMotorRange(output[FourWheel::back_right]));
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::CanBridgeUpdateEventXYWrapper(communication::can_bridge_ext_id_t ext_id,
                                         communication::can_bridge_data_t data, void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventXY(ext_id,data);
    }

    void Chassis::CanBridgeUpdateEventTurnWrapper(communication::can_bridge_ext_id_t ext_id,
                                           communication::can_bridge_data_t data, void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventTurn(ext_id,data);
    }

    void Chassis::CanBridgeUpdateEventPowerLimitWrapper(communication::can_bridge_ext_id_t ext_id,
                                                 communication::can_bridge_data_t data,
                                                 void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventPowerLimit(ext_id,data);
    }

    void Chassis::CanBridgeUpdateEventCurrentPowerWrapper(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data,
                                                   void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventCurrentPower(ext_id,data);
    }

    void Chassis::CanBridgeUpdateEventXY(communication::can_bridge_ext_id_t ext_id,
                                         communication::can_bridge_data_t data) {
        if(can_bridge_tx_id_!=0x00){
            if(can_bridge_tx_id_!=ext_id.data.tx_id){
                return;
            }
        }
        can_bridge_vx_ = data.data_two_float.data[0];
        can_bridge_vy_ = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeUpdateEventTurn(communication::can_bridge_ext_id_t ext_id,
                                           communication::can_bridge_data_t data) {
        if(can_bridge_tx_id_!=0x00){
            if(can_bridge_tx_id_!=ext_id.data.tx_id){
                return;
            }
        }
        can_bridge_vt_ = data.data_two_float.data[1];
        float is_enable = data.data_two_float.data[0];
        if(is_enable>0.1f){
            chassis_enable_ = true;
        }else{
            chassis_enable_ = false;
        }
        if(chassis_enable_){
            SetSpeed(can_bridge_vx_,can_bridge_vy_,can_bridge_vt_);
        }else{
            SetSpeed(0,0,0);
        }
    }
    void Chassis::CanBridgeUpdateEventPowerLimit(communication::can_bridge_ext_id_t ext_id,
                                                 communication::can_bridge_data_t data) {
        if(can_bridge_tx_id_!=0x00){
            if(can_bridge_tx_id_!=ext_id.data.tx_id){
                return;
            }
        }
        float is_enable = data.data_two_float.data[0];
        if(is_enable>0.1f){
            power_limit_on_ = true;
        }else{
            power_limit_on_ = false;
        }
        power_limit_info_.power_limit = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data) {
        if(can_bridge_tx_id_!=0x00){
            if(can_bridge_tx_id_!=ext_id.data.tx_id){
                return;
            }
        }
        current_chassis_power_ = data.data_two_float.data[0];
        current_chassis_power_buffer_ = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeSetTxId(uint8_t tx_id) {
        can_bridge_tx_id_ = tx_id;
    }


} /* namespace control */
