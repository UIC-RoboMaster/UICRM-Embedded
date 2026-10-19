/*###########################################################
 # Copyright (c) 2026. BNU-HKBU UIC RoboMaster              #
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

#include "dm_imu.h"

namespace {

    constexpr float kDegreesToRadians = 0.017453292519943295f;

    /**
     * @brief 按小端序解码一个 16 位无符号整数。
     * @param[in] data 至少包含 2 字节的输入数据。
     * @return 解码后的无符号整数。
     */
    uint16_t ReadUint16(const uint8_t* data) {
        return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    }

    /**
     * @brief 将 16 位无符号原始值线性映射到指定物理量范围。
     * @param raw 模块返回的原始值，范围为 0～65535。
     * @param min 对应原始值 0 的物理量下限。
     * @param max 对应原始值 65535 的物理量上限。
     * @return 解码后的物理量，单位与传入的范围一致。
     */
    float Decode16(uint16_t raw, float min, float max) {
        return static_cast<float>(raw) * (max - min) / 65535.0f + min;
    }

    /**
     * @brief 将一个 14 位四元数原始分量转换为浮点值。
     * @param raw 解包后的原始分量，范围为 0～16383。
     * @return 范围为 [-1, 1] 的无量纲四元数分量。
     */
    float DecodeQuaternion(uint16_t raw) {
        return static_cast<float>(raw) * 2.0f / 16383.0f - 1.0f;
    }

}  // namespace

namespace imu {

    DMIMU::DMIMU(DMIMU_init_t init) : DMIMU(init.can, init.tx_id, init.rx_id, init.online_threshold) {
    }

    DMIMU::DMIMU(bsp::CAN* can, uint16_t tx_id, uint16_t rx_id, uint32_t online_threshold)
        : ConnectionDriver(online_threshold), can_(can), tx_id_(tx_id) {
        RM_ASSERT_TRUE(can_ != nullptr, "DMIMU requires a CAN manager");
        RM_ASSERT_TRUE(tx_id <= 0x7ff && rx_id <= 0x7ff, "DMIMU requires standard CAN IDs");
        RM_ASSERT_EQ(
            can_->RegisterRxCallback(rx_id, CallbackWrapper, this),
            0,
            "Cannot register DMIMU feedback callback"
        );
    }

    bool DMIMU::SendCommand(DMIMURegister reg, bool write, uint32_t value) {
        // Serialize the 32-bit parameter explicitly in little-endian order.
        const uint8_t data[8] = {
            0xcc,
            static_cast<uint8_t>(reg),
            static_cast<uint8_t>(write),
            0xdd,
            static_cast<uint8_t>(value),
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value >> 16),
            static_cast<uint8_t>(value >> 24),
        };
        // H7 returns an encoded DLC on success, whereas F1/F4 return a byte count.
        return can_->Transmit(tx_id_, data, sizeof(data)) >= 0;
    }

    bool DMIMU::ReadReg(DMIMURegister reg) {
        return SendCommand(reg, false, 0);
    }

    bool DMIMU::WriteReg(DMIMURegister reg, uint32_t value) {
        return SendCommand(reg, true, value);
    }

    bool DMIMU::getAccel() {
        return ReadReg(DMIMURegister::ACCEL);
    }

    bool DMIMU::getGyro() {
        return ReadReg(DMIMURegister::GYRO);
    }

    bool DMIMU::getEuler() {
        return ReadReg(DMIMURegister::EULER);
    }

    bool DMIMU::getQuaternion() {
        return ReadReg(DMIMURegister::QUATERNION);
    }

    bool DMIMU::SetActiveMode(bool active) {
        return WriteReg(DMIMURegister::ACTIVE_MODE, active ? 1 : 0);
    }

    bool DMIMU::SetActiveDelay(uint32_t delay_ms) {
        if (delay_ms < 1 || delay_ms > 1000)
            return false;
        return WriteReg(DMIMURegister::ACTIVE_DELAY, delay_ms);
    }

    bool DMIMU::SaveParameters() {
        return WriteReg(DMIMURegister::SAVE_PARAMETERS, 0);
    }

    DMIMU_data_t DMIMU::GetData() const {
        DMIMU_data_t snapshot;
        const uint32_t interrupt_mask = __get_PRIMASK();
        __disable_irq();
        for (unsigned i = 0; i < 4; ++i)
            snapshot.INS_quat[i] = INS_quat[i];
        for (unsigned i = 0; i < 3; ++i) {
            snapshot.INS_angle[i] = INS_angle[i];
            snapshot.INS_accel[i] = INS_accel[i];
            snapshot.INS_gyro[i] = INS_gyro[i];
        }
        snapshot.Temp = Temp;
        snapshot.received_mask = received_mask_;
        snapshot.update_count = update_count_;
        __set_PRIMASK(interrupt_mask);
        return snapshot;
    }

    void DMIMU::CallbackWrapper(const uint8_t data[], void* args) {
        auto* imu = static_cast<DMIMU*>(args);
        imu->UpdateData(data);
    }

    void DMIMU::UpdateData(const uint8_t data[]) {
        uint8_t received = 0;
        switch (data[0]) {
            case 1:
                for (unsigned i = 0; i < 3; ++i)
                    INS_accel[i] = Decode16(ReadUint16(data + 2 + 2 * i), -235.2f, 235.2f);
                Temp = data[1];
                received = ACCEL_RECEIVED;
                break;
            case 2:
                for (unsigned i = 0; i < 3; ++i)
                    INS_gyro[i] = Decode16(ReadUint16(data + 2 + 2 * i), -34.88f, 34.88f);
                received = GYRO_RECEIVED;
                break;
            case 3:
                // The wire order is pitch, yaw, roll in degrees; IMU_typeC exposes yaw, pitch, roll in radians.
                INS_angle[0] = Decode16(ReadUint16(data + 4), -180.0f, 180.0f) * kDegreesToRadians;
                INS_angle[1] = Decode16(ReadUint16(data + 2), -90.0f, 90.0f) * kDegreesToRadians;
                INS_angle[2] = Decode16(ReadUint16(data + 6), -180.0f, 180.0f) * kDegreesToRadians;
                received = EULER_RECEIVED;
                break;
            case 4: {
                // Four consecutive 14-bit fields occupy bytes 1..7, most significant bits first.
                // W needs all six high bits of byte 2 (0xfc, correcting 0xf8 in the vendor example).
                const uint16_t w = (static_cast<uint16_t>(data[1]) << 6) | ((data[2] & 0xfc) >> 2);
                const uint16_t x = ((data[2] & 0x03) << 12) | (data[3] << 4) | (data[4] >> 4);
                const uint16_t y = ((data[4] & 0x0f) << 10) | (data[5] << 2) | (data[6] >> 6);
                const uint16_t z = ((data[6] & 0x3f) << 8) | data[7];
                INS_quat[0] = DecodeQuaternion(w);
                INS_quat[1] = DecodeQuaternion(x);
                INS_quat[2] = DecodeQuaternion(y);
                INS_quat[3] = DecodeQuaternion(z);
                received = QUATERNION_RECEIVED;
                break;
            }
            default:
                // Configuration replies and unrelated payloads are not measurement heartbeats.
                return;
        }

        received_mask_ = received_mask_ | received;
        update_count_ = update_count_ + 1;
        Heartbeat();
    }

}  // namespace imu
