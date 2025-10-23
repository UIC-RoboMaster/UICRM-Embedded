// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by RinChord on 2025/10/21.
//

#include "AS5600.h"

namespace imu
{
    AS5600::AS5600(as5600_init_t init) {
        i2c_addr_ = init.i2c_addr;
        i2c_ = init.hi2c;
        Init();
    }
    void AS5600::Init() {

        if (!IsReady()) {
            angle_ = -1;
            return;
        }
        ReadAngle();
        power_on_angle_ = angle_;
    }

    void AS5600::Cailbrate() {
        ReadRawAngle();
    }

    void AS5600::ReadAngle() {
        uint8_t recieve[2];
        i2c_->MemoryRead(i2c_addr_ << 1, 0x0C, recieve, 2);
        angle_ = recieve[0] << 8 | recieve[1];
   }

    void AS5600::ReadRawAngle() {
        uint8_t recieve[2];
        i2c_->MemoryRead(i2c_addr_ << 1, 0x0E, recieve, 2);
        rawangle_ = recieve[0] << 8 | recieve[1];
   }

    void AS5600::Burn() {

    }

    void AS5600::ReadConfig() {

    }

    bool AS5600::IsReady() {
        return i2c_->isReady(i2c_addr_);
    }

    uint16_t AS5600::GetAngle() {
        return angle_;
    }
    uint16_t AS5600::GetRawAngle() {
        return rawangle_;
    }


    void AS5600::Update() {
        ReadRawAngle();
        ReadAngle();
        relative_angle_ = angle_ - power_on_angle_;
    }

} // AS5600

