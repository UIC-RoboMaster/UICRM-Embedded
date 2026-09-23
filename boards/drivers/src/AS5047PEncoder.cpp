/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

#include "AS5047PEncoder.h"

#include "arm_math.h"
#include "utils.h"

namespace driver {

    namespace {
        /* 模块以 1kHz 发送，连续丢 20 帧判掉线 */
        constexpr uint32_t ONLINE_THRESHOLD_MS = 20;
        constexpr float COUNT_TO_RAD = 2 * PI / AS5047PEncoder::COUNTS_PER_REV;
        constexpr float RPM_TO_RAD_PER_SEC = 2 * PI / 60.0f;
    }  // namespace

    AS5047PEncoder::AS5047PEncoder(const as5047p_encoder_init_t& init)
        : ConnectionDriver(ONLINE_THRESHOLD_MS),
          can_(init.can),
          rx_id_(init.rx_id),
          offset_(init.offset),
          reversed_(init.reversed) {
        can_->RegisterRxCallback(rx_id_, CallbackWrapper, this);
    }

    void AS5047PEncoder::CallbackWrapper(const uint8_t data[], void* args) {
        AS5047PEncoder* encoder = reinterpret_cast<AS5047PEncoder*>(args);
        encoder->UpdateData(data);
    }

    void AS5047PEncoder::UpdateData(const uint8_t data[]) {
        raw_count_ = ((uint16_t)data[0] << 8 | (uint16_t)data[1]) & (COUNTS_PER_REV - 1);
        raw_rpm_ = (int16_t)((uint16_t)data[2] << 8 | (uint16_t)data[3]);
        raw_cumulated_count_ = (int32_t)((uint32_t)data[4] << 24 | (uint32_t)data[5] << 16 |
                                         (uint32_t)data[6] << 8 | (uint32_t)data[7]);

        Heartbeat();
    }

    float AS5047PEncoder::RawToRad() const {
        const float angle = raw_count_ * COUNT_TO_RAD;
        return reversed_ ? -angle : angle;
    }

    uint16_t AS5047PEncoder::GetRawCount() const {
        return raw_count_;
    }

    float AS5047PEncoder::GetAngle() const {
        return wrapStrict<float>(RawToRad() - offset_, 0, 2 * PI);
    }

    float AS5047PEncoder::GetAngleWrapped() const {
        return wrapStrict<float>(RawToRad() - offset_, -PI, PI);
    }

    float AS5047PEncoder::GetOmega() const {
        const float omega = raw_rpm_ * RPM_TO_RAD_PER_SEC;
        return reversed_ ? -omega : omega;
    }

    float AS5047PEncoder::GetRpm() const {
        return reversed_ ? -raw_rpm_ : raw_rpm_;
    }

    float AS5047PEncoder::GetCumulatedAngle() const {
        const float angle = raw_cumulated_count_ * COUNT_TO_RAD;
        return reversed_ ? -angle : angle;
    }

    int32_t AS5047PEncoder::GetRawCumulatedCount() const {
        return reversed_ ? -raw_cumulated_count_ : raw_cumulated_count_;
    }

    void AS5047PEncoder::SetOffset(float offset) {
        offset_ = offset;
    }

    float AS5047PEncoder::GetOffset() const {
        return offset_;
    }

    void AS5047PEncoder::AlignHere() {
        offset_ = RawToRad();
    }

}  // namespace driver
