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

#pragma once
#include "main.h"
#include "bsp_i2c.h"
#include "bsp_gpio.h"
#include "imu_info.h"

#define IST8310_WHO_AM_I 0x00        // ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE 0x10  // device ID

#define IST8310_WRITE_REG_NUM 4

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

#define IST8310_RX_BUF_DATA_OFFSET 16

// the first column:the registers of IST8310.
// the second column: the value to be writed to the registers.
// the third column: return error value.
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},   // enalbe interrupt  and low pin polarity.
    {0x41, 0x09, 0x02},   // average 2 times.
    {0x42, 0xC0, 0x03},   // must be 0xC0.
    {0x0A, 0x0B, 0x04}};  // 200Hz output rate.

#define IST8310_IIC_ADDRESS 0x0E  // the I2C address of IST8310

namespace imu{
    typedef struct {
        bsp::I2C* i2c;
        uint16_t int_pin;
        GPIO_TypeDef* rst_group;
        uint16_t rst_pin;
    } IST8310_init_t;

    typedef struct {
        uint8_t status;
        float mag[3];
    } IST8310_real_data_t;

    typedef void (*ist8310_callback_t)(float mag[3]);


    class IST8310 : public bsp::GPIT {
      public:
        IST8310(IST8310_init_t init);
        IST8310(bsp::I2C* i2c, uint16_t int_pin, GPIO_TypeDef* rst_group,
                uint16_t rst_pin);
        bool IsReady();
        void RegisterCallback(ist8310_callback_t callback);
        void ist8310_read_over(uint8_t* status_buf, IST8310_real_data_t* ist8310_real_data);
        float mag[3];

      private:
        uint8_t Init();
        void ist8310_read_mag(float mag_[3]);
        void IntCallback() final;


        void ist8310_RST_H();
        void ist8310_RST_L();
        uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
        void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
        void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);
        void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t* data, uint8_t len);

        bsp::I2C* i2c_;
        GPIO_TypeDef* rst_group_;
        uint16_t rst_pin_;

        ist8310_callback_t callback_ = [](float mag_tmp[3]) {UNUSED(mag_tmp);};
    };
}