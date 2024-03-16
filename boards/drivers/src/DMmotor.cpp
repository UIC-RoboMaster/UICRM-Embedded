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

#include "DMmotor.h"

namespace driver {
    /**
     * @brief standard can motor callback, used to update motor data
     *
     * @param data data that come from motor
     * @param args pointer to a MotorCANBase instance
     */
    static void can_motor_4310_callback(const uint8_t data[], void* args) {
        DMMotor4310* motor = reinterpret_cast<DMMotor4310*>(args);
        motor->UpdateData(data);
    }

    DMMotor4310::DMMotor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_m4310_mode_t mode)
        : ConnectionDriver(50),can_(can), rx_id_(rx_id), tx_id_(tx_id) {
        can->RegisterRxCallback(rx_id, can_motor_4310_callback, this);
        /* following the CAN id format from the m4310 V2.1 document */
        mode_ = mode;
        if (mode == MIT) {
            tx_id_actual_ = tx_id;
        } else if (mode == POS_VEL) {
            tx_id_actual_ = tx_id + 0x100;
        } else if (mode == VEL) {
            tx_id_actual_ = tx_id + 0x200;
        } else {
            RM_EXPECT_TRUE(false, "Invalid mode number!");
        }
    }

    void DMMotor4310::MotorEnable() {
        uint8_t data[8] = {0};
        data[0] = 0xff;
        data[1] = 0xff;
        data[2] = 0xff;
        data[3] = 0xff;
        data[4] = 0xff;
        data[5] = 0xff;
        data[6] = 0xff;
        data[7] = 0xfc;
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor4310::MotorDisable() {
        uint8_t data[8] = {0};
        data[0] = 0xff;
        data[1] = 0xff;
        data[2] = 0xff;
        data[3] = 0xff;
        data[4] = 0xff;
        data[5] = 0xff;
        data[6] = 0xff;
        data[7] = 0xfd;
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor4310::SetZeroPos() {
        uint8_t data[8] = {0};
        data[0] = 0xff;
        data[1] = 0xff;
        data[2] = 0xff;
        data[3] = 0xff;
        data[4] = 0xff;
        data[5] = 0xff;
        data[6] = 0xff;
        data[7] = 0xfe;
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor4310::SetOutput(float position, float velocity, float kp, float kd, float torque) {
        kp_set_ = kp;
        kd_set_ = kd;
        pos_set_ = position;
        vel_set_ = velocity;
        torque_set_ = torque;
    }

    void DMMotor4310::SetOutput(float position, float velocity) {
        pos_set_ = position;
        vel_set_ = velocity;
    }

    void DMMotor4310::SetOutput(float velocity) {
        vel_set_ = velocity;
    }

    void DMMotor4310::TransmitOutput() {
        uint8_t data[8] = {0};
        int16_t kp_tmp, kd_tmp, pos_tmp, vel_tmp, torque_tmp;

        if (mode_ == MIT) {
            // converting float to unsigned int before transmitting
            kp_tmp = float_to_uint(kp_set_, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(kd_set_, KD_MIN, KD_MAX, 12);
            pos_tmp = float_to_uint(pos_set_, P_MIN, V_MAX, 16);
            vel_tmp = float_to_uint(vel_set_, V_MIN, V_MAX, 12);
            torque_tmp = float_to_uint(torque_set_, T_MIN, T_MAX, 12);
            data[0] = pos_tmp >> 8;
            data[1] = pos_tmp & 0x00ff;
            data[2] = (vel_tmp >> 4) & 0x00ff;
            data[3] = ((vel_tmp & 0x000f) << 4) | ((kp_tmp >> 8) & 0x000f);
            data[4] = kp_tmp & 0x00ff;
            data[5] = (kd_tmp >> 4) & 0x00ff;
            data[6] = ((kd_tmp & 0x000f) << 4) | ((torque_tmp >> 8) & 0x000f);
            data[7] = torque_tmp & 0x00ff;
        } else if (mode_ == POS_VEL) {
            uint8_t *pbuf, *vbuf;
            pbuf = (uint8_t*)&pos_set_;
            vbuf = (uint8_t*)&vel_set_;
            data[0] = *pbuf;
            data[1] = *(pbuf + 1);
            data[2] = *(pbuf + 2);
            data[3] = *(pbuf + 3);
            data[4] = *vbuf;
            data[5] = *(vbuf + 1);
            data[6] = *(vbuf + 2);
            data[7] = *(vbuf + 3);
        } else if (mode_ == VEL) {
            uint8_t* vbuf;
            vbuf = (uint8_t*)&vel_set_;
            data[0] = *vbuf;
            data[1] = *(vbuf + 1);
            data[2] = *(vbuf + 2);
            data[3] = *(vbuf + 3);
        } else {
            RM_EXPECT_TRUE(false, "Invalid mode number!");
        }
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor4310::UpdateData(const uint8_t data[]) {
        raw_pos_ = data[1] << 8 | data[2];
        raw_vel_ = data[3] << 4 | (data[4] & 0xf0) >> 4;
        raw_torque_ = data[5] | (data[4] & 0x0f) << 8;
        raw_mosTemp_ = data[6];
        raw_motorTemp_ = data[7];

        theta_ = uint_to_float(raw_pos_, P_MIN, P_MAX, 16);
        omega_ = uint_to_float(raw_vel_, V_MIN, V_MAX, 12);
        torque_ = uint_to_float(raw_torque_, T_MIN, T_MAX, 12);

        Heartbeat();
    }

    void DMMotor4310::PrintData() {
        set_cursor(0, 0);
        clear_screen();
        print("Position: % .4f ", GetTheta());
        print("Velocity: % .4f ", GetOmega());
        print("Torque: % .4f ", GetTorque());
        print("Rotor temp: % .4f \r\n", raw_motorTemp_);
    }

    uint16_t DMMotor4310::float_to_uint(float x, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float DMMotor4310::uint_to_float(int x_int, float x_min, float x_max, int bits)

    {
        /// converts unsigned int to float, given range and number of bits ///

        float span = x_max - x_min;

        float offset = x_min;

        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }
    float DMMotor4310::GetTheta() const {
        return theta_;
    }

    float DMMotor4310::GetOmega() const {
        return omega_;
    }
    float DMMotor4310::GetTorque() const {
        return torque_;
    }
}  // namespace driver