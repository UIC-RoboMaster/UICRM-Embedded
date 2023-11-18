
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

#include <map>

#include "bsp_gpio.h"
#include "bsp_heater.h"
#include "cmsis_os.h"
#include "spi.h"

// acc (6 bytes) + temp (2 bytes) + gyro (6 bytes) + mag (6 bytes)
#define MPU6500_SIZEOF_DATA 20

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

#define IST8310_RX_BUF_DATA_OFFSET 16

#define INS_TASK_INIT_TIME 7

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

namespace bsp {

    typedef struct {
        float x;
        float y;
        float z;
    } vec3f_t;

    class MPU6500 : public GPIT {
      public:
        /**
         * @brief constructor for a MPU6500 IMU sensor
         *
         * @param hspi         HAL SPI handle associated with the sensor
         * @param chip_select  chip select gpio pin
         * @param int_pin      interrupt pin number
         */
        MPU6500(SPI_HandleTypeDef* hspi, const GPIO& chip_select, uint16_t int_pin);

        /**
         * @brief reset sensor registers
         */
        void Reset();

        // 3-axis accelerometer
        vec3f_t acce;
        // 3-axis gyroscope
        vec3f_t gyro;
        // 3-axis magnetometer
        vec3f_t mag;
        // sensor temperature
        float temp;
        // sensor timestamp
        uint64_t timestamp = 0;

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
        void IntCallback() override final;

        SPI_HandleTypeDef* hspi_;
        GPIO chip_select_;

        uint8_t io_buff_[MPU6500_SIZEOF_DATA + 1];  // spi tx+rx buffer

        // global interrupt wrapper
        // TODO(alvin): try to support multiple instances in the future
        static void SPITxRxCpltCallback(SPI_HandleTypeDef* hspi);
        static MPU6500* mpu6500;
    };

    typedef struct {
        I2C_HandleTypeDef* hi2c;
        uint16_t int_pin;
        GPIO_TypeDef* rst_group;
        uint16_t rst_pin;
    } IST8310_init_t;

    typedef struct {
        uint8_t status;
        float mag[3];
    } IST8310_real_data_t;

    class IST8310 : public GPIT {
      public:
        IST8310(IST8310_init_t init);
        IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, GPIO_TypeDef* rst_group,
                uint16_t rst_pin);
        bool IsReady();
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

        I2C_HandleTypeDef* hi2c_;
        GPIO_TypeDef* rst_group_;
        uint16_t rst_pin_;
    };

} /* namespace bsp */
