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
#include <math.h>

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "imu_info.h"
#include "main.h"
#define MPU6500_DELAY 55  // SPI delay
// configured with initialization sequences
#define MPU6500_ACC_FACTOR 4096.0f
#define MPU6500_GYRO_FACTOR 32.768f

// acc (6 bytes) + temp (2 bytes) + gyro (6 bytes) + mag_ (6 bytes)
#define MPU6500_SIZEOF_DATA 20

#define MPU6500_SELF_TEST_XG 0x00
#define MPU6500_SELF_TEST_YG 0x01
#define MPU6500_SELF_TEST_ZG 0x02
#define MPU6500_SELF_TEST_XA 0x0D
#define MPU6500_SELF_TEST_YA 0x0E
#define MPU6500_SELF_TEST_ZA 0x0F
#define MPU6500_XG_OFFSET_H 0x13
#define MPU6500_XG_OFFSET_L 0x14
#define MPU6500_YG_OFFSET_H 0x15
#define MPU6500_YG_OFFSET_L 0x16
#define MPU6500_ZG_OFFSET_H 0x17
#define MPU6500_ZG_OFFSET_L 0x18
#define MPU6500_SMPLRT_DIV 0x19
#define MPU6500_CONFIG 0x1A
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_CONFIG_2 0x1D
#define MPU6500_LP_ACCEL_ODR 0x1E
#define MPU6500_MOT_THR 0x1F
#define MPU6500_FIFO_EN 0x23
#define MPU6500_I2C_MST_CTRL 0x24
#define MPU6500_I2C_SLV0_ADDR 0x25
#define MPU6500_I2C_SLV0_REG 0x26
#define MPU6500_I2C_SLV0_CTRL 0x27
#define MPU6500_I2C_SLV1_ADDR 0x28
#define MPU6500_I2C_SLV1_REG 0x29
#define MPU6500_I2C_SLV1_CTRL 0x2A
#define MPU6500_I2C_SLV2_ADDR 0x2B
#define MPU6500_I2C_SLV2_REG 0x2C
#define MPU6500_I2C_SLV2_CTRL 0x2D
#define MPU6500_I2C_SLV3_ADDR 0x2E
#define MPU6500_I2C_SLV3_REG 0x2F
#define MPU6500_I2C_SLV3_CTRL 0x30
#define MPU6500_I2C_SLV4_ADDR 0x31
#define MPU6500_I2C_SLV4_REG 0x32
#define MPU6500_I2C_SLV4_DO 0x33
#define MPU6500_I2C_SLV4_CTRL 0x34
#define MPU6500_I2C_SLV4_DI 0x35
#define MPU6500_I2C_MST_STATUS 0x36
#define MPU6500_INT_PIN_CFG 0x37
#define MPU6500_INT_ENABLE 0x38
#define MPU6500_INT_STATUS 0x3A
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_ACCEL_XOUT_L 0x3C
#define MPU6500_ACCEL_YOUT_H 0x3D
#define MPU6500_ACCEL_YOUT_L 0x3E
#define MPU6500_ACCEL_ZOUT_H 0x3F
#define MPU6500_ACCEL_ZOUT_L 0x40
#define MPU6500_TEMP_OUT_H 0x41
#define MPU6500_TEMP_OUT_L 0x42
#define MPU6500_GYRO_XOUT_H 0x43
#define MPU6500_GYRO_XOUT_L 0x44
#define MPU6500_GYRO_YOUT_H 0x45
#define MPU6500_GYRO_YOUT_L 0x46
#define MPU6500_GYRO_ZOUT_H 0x47
#define MPU6500_GYRO_ZOUT_L 0x48
#define MPU6500_EXT_SENS_DATA_00 0x49
#define MPU6500_EXT_SENS_DATA_01 0x4A
#define MPU6500_EXT_SENS_DATA_02 0x4B
#define MPU6500_EXT_SENS_DATA_03 0x4C
#define MPU6500_EXT_SENS_DATA_04 0x4D
#define MPU6500_EXT_SENS_DATA_05 0x4E
#define MPU6500_EXT_SENS_DATA_06 0x4F
#define MPU6500_EXT_SENS_DATA_07 0x50
#define MPU6500_EXT_SENS_DATA_08 0x51
#define MPU6500_EXT_SENS_DATA_09 0x52
#define MPU6500_EXT_SENS_DATA_10 0x53
#define MPU6500_EXT_SENS_DATA_11 0x54
#define MPU6500_EXT_SENS_DATA_12 0x55
#define MPU6500_EXT_SENS_DATA_13 0x56
#define MPU6500_EXT_SENS_DATA_14 0x57
#define MPU6500_EXT_SENS_DATA_15 0x58
#define MPU6500_EXT_SENS_DATA_16 0x59
#define MPU6500_EXT_SENS_DATA_17 0x5A
#define MPU6500_EXT_SENS_DATA_18 0x5B
#define MPU6500_EXT_SENS_DATA_19 0x5C
#define MPU6500_EXT_SENS_DATA_20 0x5D
#define MPU6500_EXT_SENS_DATA_21 0x5E
#define MPU6500_EXT_SENS_DATA_22 0x5F
#define MPU6500_EXT_SENS_DATA_23 0x60
#define MPU6500_I2C_SLV0_DO 0x63
#define MPU6500_I2C_SLV1_DO 0x64
#define MPU6500_I2C_SLV2_DO 0x65
#define MPU6500_I2C_SLV3_DO 0x66
#define MPU6500_I2C_MST_DELAY_CTRL 0x67
#define MPU6500_SIGNAL_PATH_RESET 0x68
#define MPU6500_MOT_DETECT_CTRL 0x69
#define MPU6500_USER_CTRL 0x6A
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_PWR_MGMT_2 0x6C
#define MPU6500_FIFO_COUNTH 0x72
#define MPU6500_FIFO_COUNTL 0x73
#define MPU6500_FIFO_R_W 0x74
#define MPU6500_WHO_AM_I 0x75
#define MPU6500_XA_OFFSET_H 0x77
#define MPU6500_XA_OFFSET_L 0x78
#define MPU6500_YA_OFFSET_H 0x7A
#define MPU6500_YA_OFFSET_L 0x7B
#define MPU6500_ZA_OFFSET_H 0x7D
#define MPU6500_ZA_OFFSET_L 0x7E

#define MPU6500_TEMP_OFFSET 21      // datasheet p12
#define MPU6500_TEMP_FACTOR 333.87  // datasheet p12

#define MPU6500_ID 0x70

#define MPU6500_IIC_ADDR 0x68

namespace imu {

    typedef struct {
        bsp::SPIMaster* spi;
        bsp::GPIO* cs;
        bsp::GPIT* int_pin;
        bool use_mag = false;
        bool dma = true;
    } mpu6500_init_t;


    typedef void (*mpu6500_callback_t)();

    class MPU6500 {
      public:
        /**
         * @brief constructor for a MPU6500 IMU sensor
         *
         * @param hspi         HAL SPI handle associated with the sensor
         * @param chip_select  chip select gpio pin
         * @param int_pin      interrupt pin number
         */
        MPU6500(mpu6500_init_t init);

        /**
         * @brief reset sensor registers
         */
        void Reset();

        float gyro_[3];
        float accel_[3];
        float mag_[3];
        float temperature_;
        float time_;

        void RegisterCallback(mpu6500_callback_t callback);

      private:
        /**
         * @brief sample latest sensor data
         */
        void UpdateData();

        void IST8310Init();
        void WriteReg(uint8_t reg, uint8_t data);
        void WriteRegs(uint8_t reg_start, uint8_t* data, uint8_t len);
        void ReadReg(uint8_t reg, uint8_t* data);
        void ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len);

        void SPITxRxCpltCallback();
        static void IntCallback(void* args);

        bsp::SPIMaster* spi_;
        bsp::SPIDevice* spi_device_;
        bsp::GPIO* cs_;
        bsp::GPIT* int_pin_;

        bool use_mag_;
        bool dma_;

        uint8_t io_buff_[MPU6500_SIZEOF_DATA + 1];  // spi tx+rx buffer

        mpu6500_callback_t callback_=[](){};

        // global interrupt wrapper
        static void SPITxRxCpltCallbackWrapper(void* args);
    };
};  // namespace imu