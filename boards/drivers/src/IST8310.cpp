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

#include "IST8310.h"

namespace imu {
    IST8310::IST8310(IST8310_init_t init) : GPIT(init.int_pin) {
        i2c_ = init.i2c;
        rst_group_ = init.rst_group;
        rst_pin_ = init.rst_pin;
        RM_ASSERT_TRUE(Init() == IST8310_NO_ERROR, "IST8310 init error");
    }

    IST8310::IST8310(bsp::I2C* i2c, uint16_t int_pin, GPIO_TypeDef* rst_group, uint16_t rst_pin)
        : GPIT(int_pin) {
        i2c_ = i2c;
        rst_group_ = rst_group;
        rst_pin_ = rst_pin;
        RM_ASSERT_TRUE(Init() == IST8310_NO_ERROR, "IST8310 init error");
    }

    bool IST8310::IsReady() {
        return i2c_->isReady(IST8310_IIC_ADDRESS << 1, 100);
    }

    void IST8310::RegisterCallback(imu::ist8310_callback_t callback) {
        callback_ = callback;
    }

    uint8_t IST8310::Init() {
        const uint8_t wait_time = 1;
        const uint8_t sleepTime = 50;
        uint8_t res;
        uint8_t writeNum;

        ist8310_RST_L();
        HAL_Delay(sleepTime);
        ist8310_RST_H();
        HAL_Delay(sleepTime);

        res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
        if (res != IST8310_WHO_AM_I_VALUE)
            return IST8310_NO_SENSOR;

        // set mpu6500 sonsor config and check
        for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
            ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
                                         ist8310_write_reg_data_error[writeNum][1]);
            HAL_Delay(wait_time);
            res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
            HAL_Delay(wait_time);
            if (res != ist8310_write_reg_data_error[writeNum][1])
                return ist8310_write_reg_data_error[writeNum][2];
        }
        return IST8310_NO_ERROR;
    }

    void IST8310::ist8310_read_over(uint8_t* status_buf, IST8310_real_data_t* ist8310_real_data) {
        if (status_buf[0] & 0x01) {
            int16_t temp_ist8310_data = 0;
            ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

            temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
            ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
            temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
            ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
            temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
            ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
        } else {
            ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
        }
    }

    void IST8310::ist8310_read_mag(float mag_[3]) {
        uint8_t buf[6];
        int16_t temp_ist8310_data = 0;
        // read the "DATAXL" register (0x03)
        ist8310_IIC_read_muli_reg(0x03, buf, 6);

        temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
        mag_[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
        mag_[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
        mag_[2] = MAG_SEN * temp_ist8310_data;
    }

    void IST8310::IntCallback() {
        ist8310_read_mag(mag);
        callback_(mag);
    }

    void IST8310::ist8310_RST_H() {
        HAL_GPIO_WritePin(rst_group_, rst_pin_, GPIO_PIN_SET);
    }

    void IST8310::ist8310_RST_L() {
        HAL_GPIO_WritePin(rst_group_, rst_pin_, GPIO_PIN_RESET);
    }

    uint8_t IST8310::ist8310_IIC_read_single_reg(uint8_t reg) {
        uint8_t res = 0;
        i2c_->MemoryRead(IST8310_IIC_ADDRESS << 1, reg, &res, 1);
        return res;
    }

    void IST8310::ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
        i2c_->MemoryWrite(IST8310_IIC_ADDRESS << 1, reg, &data, 1);
    }

    void IST8310::ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        i2c_->MemoryRead(IST8310_IIC_ADDRESS << 1, reg, buf, len);
    }

    void IST8310::ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t* data, uint8_t len) {
        i2c_->MemoryWrite(IST8310_IIC_ADDRESS << 1, reg, data, len);
    }
}  // namespace imu