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

#include "cybergear.h"

#include <string.h>

#include "arm_math.h"
#include "bsp_print.h"
#include "utils.h"
namespace driver {

    /**
     * @brief standard can motor callback, used to update motor data
     *
     * @param data data that come from motor
     * @param args pointer to a MotorCANBase instance
     */
    static void cybergear_motor_callback(const uint8_t data[], const uint32_t ext_id, void* args) {
        CyberGear* motor = reinterpret_cast<CyberGear*>(args);
        motor->UpdateData(data, ext_id);
    }

    int16_t CyberGear::float_to_uint(float x, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        if (x > x_max)
            x = x_max;
        else if (x < x_min)
            x = x_min;
        return (int16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }
    float CyberGear::uint_to_float(int x, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return (float)(x * ((float)span / (float)((1 << bits) - 1)) + offset);
    }
    CyberGear::CyberGear(bsp::CAN* can, uint8_t tx_id, uint8_t rx_id) {
        can_ = can;
        tx_id_ = tx_id;
        rx_id_ = rx_id;
        tx_ext_id_.id = tx_id;
        can_->RegisterRxExtendCallback(rx_id_, cybergear_motor_callback, this);
    }
    CyberGear::~CyberGear() = default;
    void CyberGear::TransmitData() {
        // transfer tx_ext_id_ to uint32_t
        uint32_t tx_ext_id = 0;
        memcpy(&tx_ext_id, &tx_ext_id_, sizeof(tx_ext_id_));
        can_->TransmitExtend(tx_ext_id, tx_data_, 8);
    }
    void CyberGear::UpdateData(const uint8_t* data, const uint32_t ext_id) {
        memcpy(&rx_ext_id_, &ext_id, sizeof(ext_id));
        if (rx_ext_id_.id == rx_id_) {
            switch (rx_ext_id_.mode) {
                case 0:
                    // broadcast frame
                    break;
                case 2:
                    // read frame
                    mit_theta_ = uint_to_float((data[0] << 8) | data[1], -4 * PI, 4 * PI, 16);
                    theta_ = wrapc<float>(
                        uint_to_float((data[0] << 8) | data[1], -4 * PI, 4 * PI, 16), 0, 2 * PI);
                    omega_ = uint_to_float((data[2] << 8) | data[3], -30, 30, 16);
                    torque_ = uint_to_float((data[4] << 8) | data[5], -12, 12, 16);
                    temperature_ = ((data[6] << 8) | data[7]) * 0.1;
                    error_code_ = rx_ext_id_.data >> 8 & 0x3F;
                    status_ = rx_ext_id_.data >> 14 & 0x3;
                    break;
                case 17:
                    // read index
                    // TODO: add read index function
                    break;
                case 21:
                    // error handler
                    // TODO: handler error
                default:
                    break;
            }
        }
    }
    void CyberGear::MotorEnable() {
        tx_ext_id_.mode = 3;
        tx_ext_id_.data = rx_id_ & 0xFF;
        tx_ext_id_.id = tx_id_;
        memset(tx_data_, 0, sizeof(tx_data_));
        TransmitData();
    }
    void CyberGear::MotorDisable() {
        tx_ext_id_.mode = 4;
        tx_ext_id_.data = rx_id_ & 0xFF;
        tx_ext_id_.id = tx_id_;
        memset(tx_data_, 0, sizeof(tx_data_));
        TransmitData();
    }
    void CyberGear::Reset() {
        tx_ext_id_.mode = 4;
        tx_ext_id_.data = rx_id_ & 0xFF;
        tx_ext_id_.id = tx_id_;
        memset(tx_data_, 0, sizeof(tx_data_));
        tx_data_[0] = 0x01;
        TransmitData();
    }
    void CyberGear::WriteIndex(uint16_t index, uint8_t data) {
        tx_ext_id_.mode = 18;
        tx_ext_id_.data = rx_id_ & 0xFF;
        tx_ext_id_.id = tx_id_;
        memset(tx_data_, 0, sizeof(tx_data_));
        memcpy(tx_data_, &index, sizeof(index));
        memcpy(tx_data_ + 4, &data, sizeof(data));
        TransmitData();
    }
    void CyberGear::WriteIndex(uint16_t index, uint16_t data) {
        tx_ext_id_.mode = 18;
        tx_ext_id_.data = rx_id_ & 0xFF;
        tx_ext_id_.id = tx_id_;
        memset(tx_data_, 0, sizeof(tx_data_));
        memcpy(tx_data_, &index, sizeof(index));
        memcpy(tx_data_ + 4, &data, sizeof(data));
        TransmitData();
    }
    void CyberGear::WriteIndex(uint16_t index, float data) {
        tx_ext_id_.mode = 18;
        tx_ext_id_.data = rx_id_ & 0xFF;
        tx_ext_id_.id = tx_id_;
        memset(tx_data_, 0, sizeof(tx_data_));
        memcpy(tx_data_, &index, sizeof(index));
        memcpy(tx_data_ + 4, &data, sizeof(data));
        TransmitData();
    }
    void CyberGear::SetMode(cybergear_mode_e mode) {
        mode_ = mode;
        uint8_t mode_data = mode;
        WriteIndex(0x7005, mode_data);
    }
    void CyberGear::SetOutput(float output) {
        switch (mode_) {
            case CYBERGEAR_MODE_SPEED:
                WriteIndex(0x700A, output);
                break;
            case CYBERGEAR_MODE_CURRENT:
                WriteIndex(0x7006, output);
                break;
            case CYBERGEAR_MODE_POSITION:
                WriteIndex(0x7016, output);
                break;
            default:
                // Other mode needs more parameters
                break;
        }
    }
    void CyberGear::SetOutout(float position, float velocity) {
        switch (mode_) {
            case CYBERGEAR_MODE_POSITION:
                WriteIndex(0x7017, velocity);
                WriteIndex(0x7016, position);
                break;
            default:
                // Other mode needs more parameters
                break;
        }
    }
    void CyberGear::SetOutput(float position, float velocity, float torque, float kp, float kd) {
        uint16_t position_data = float_to_uint(position, -4 * PI, 4 * PI, 16);
        uint16_t velocity_data = float_to_uint(velocity, -30, 30, 16);
        uint16_t torque_data = float_to_uint(torque, -12, 12, 16);
        uint16_t kp_data = float_to_uint(kp, 0, 500, 16);
        uint16_t kd_data = float_to_uint(kd, 0, 5, 16);
        switch (mode_) {
            case CYBERGEAR_MODE_MIT:
                tx_data_[0] = position_data >> 8;
                tx_data_[1] = position_data & 0xFF;
                tx_data_[2] = velocity_data >> 8;
                tx_data_[3] = velocity_data & 0xFF;
                tx_data_[4] = kp_data >> 8;
                tx_data_[5] = kp_data & 0xFF;
                tx_data_[6] = kd_data >> 8;
                tx_data_[7] = kd_data & 0xFF;
                tx_ext_id_.id = tx_id_;
                tx_ext_id_.mode = 1;
                tx_ext_id_.data = torque_data & 0xFFFF;
                TransmitData();
                break;
            default:
                break;
        }
    }
    float CyberGear::GetTheta() const {
        return theta_;
    }
    float CyberGear::GetThetaDelta(const float target) const {
        return target - theta_;
    }
    float CyberGear::GetMITTheta() const {
        return mit_theta_;
    }
    float CyberGear::GetMITThetaDelta(const float target) const {
        return target - mit_theta_;
    }
    float CyberGear::GetOmega() const {
        return omega_;
    }
    float CyberGear::GetOmegaDelta(const float target) const {
        return target - omega_;
    }
    float CyberGear::GetTorque() const {
        return torque_;
    }
    float CyberGear::GetTemp() const {
        return temperature_;
    }
    void CyberGear::PrintData() {
        print("Position: % .4f ", GetTheta());
        print("Velocity: % .4f ", GetOmega());
        print("Torque: % .4f ", GetTorque());
        print("Motor temp: % .1f \r\n", GetTemp());
    }

};  // namespace driver