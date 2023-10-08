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

#include "BMI088.h"

namespace imu {
    BMI088::BMI088(BMI088_init_t init) {
        spi_ = init.spi;
        CS_ACCEL_ = init.CS_ACCEL;
        CS_GYRO_ = init.CS_GYRO;
        Init();
    }

    BMI088::BMI088(bsp::SPI* spi, bsp::GPIO* CS_ACCEL,bsp::GPIO* CS_GYRO) {
        spi_ = spi;
        CS_ACCEL_ = CS_ACCEL;
        CS_GYRO_ = CS_GYRO;
        Init();
    }

    bool BMI088::IsReady() {
        return true;
    }

    void BMI088::BMI088_ACCEL_NS_L() {
        CS_ACCEL_->Low();
    }

    void BMI088::BMI088_ACCEL_NS_H() {
        CS_ACCEL_->High();
    }

    void BMI088::BMI088_GYRO_NS_L() {
        CS_GYRO_->Low();
    }

    void BMI088::BMI088_GYRO_NS_H() {
        CS_GYRO_->High();
    }

    uint8_t BMI088::BMI088_read_write_byte(uint8_t tx_data) {
        uint8_t rx_data;
        spi_->TransimiReceive(&tx_data, &rx_data, 1);
        return rx_data;
    }

    void BMI088::BMI088_write_single_reg(uint8_t reg, uint8_t data) {
        BMI088_read_write_byte(reg);
        BMI088_read_write_byte(data);
    }

    void BMI088::BMI088_read_single_reg(uint8_t reg, uint8_t* data) {
        BMI088_read_write_byte(reg | 0x80);
        *data = BMI088_read_write_byte(0x55);
    }

    void BMI088::BMI088_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        BMI088_read_write_byte(reg | 0x80);
        while (len != 0) {
            *buf = BMI088_read_write_byte(0x55);
            buf++;
            len--;
        }
    }

    void BMI088::BMI088_accel_write_single_reg(uint8_t reg, uint8_t data) {
        BMI088_ACCEL_NS_L();
        BMI088_write_single_reg(reg, data);
        BMI088_ACCEL_NS_H();
    }

    void BMI088::BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data) {
        BMI088_ACCEL_NS_L();
        BMI088_read_write_byte(reg | 0x80);
        BMI088_read_write_byte(0x55);
        *data = BMI088_read_write_byte(0x55);
        BMI088_ACCEL_NS_H();
    }

    void BMI088::BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        BMI088_ACCEL_NS_L();
        BMI088_read_write_byte(reg | 0x80);
        BMI088_read_muli_reg(reg, buf, len);
        BMI088_ACCEL_NS_H();
    }

    void BMI088::BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
        BMI088_GYRO_NS_L();
        BMI088_write_single_reg(reg, data);
        BMI088_GYRO_NS_H();
    }

    void BMI088::BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data) {
        BMI088_GYRO_NS_L();
        BMI088_read_single_reg(reg, data);
        BMI088_GYRO_NS_H();
    }

    void BMI088::BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        BMI088_GYRO_NS_L();
        BMI088_read_muli_reg(reg, buf, len);
        BMI088_GYRO_NS_H();
    }

    static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
    static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

    static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
         BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL,
         BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
         BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

    static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
         BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
         BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
         BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

    uint8_t BMI088::Init() {
        uint8_t error = BMI088_NO_ERROR;

        error |= bmi088_accel_init();
        error |= bmi088_gyro_init();

        spi_->hspi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

        return error;
    }

    bool BMI088::bmi088_accel_init() {
        uint8_t res = 0;
        uint8_t write_reg_num;

        // check commiunication
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
        HAL_Delay(1);
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
        HAL_Delay(1);

        // accel software reset
        BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
        HAL_Delay(BMI088_LONG_DELAY_TIME);

        // check commiunication is normal after reset
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
        HAL_Delay(1);
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
        HAL_Delay(1);

        // check the "who am I"
        if (res != BMI088_ACC_CHIP_ID_VALUE)
            return false;

        // set accel sonsor config and check
        for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; ++write_reg_num) {
            BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0],
                                          write_BMI088_accel_reg_data_error[write_reg_num][1]);
            HAL_Delay(1);
            BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
            HAL_Delay(1);
            if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
                return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
        return false;
    }

    bool BMI088::bmi088_gyro_init() {
        uint8_t write_reg_num;
        uint8_t res = 0;

        // check commiunication
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
        HAL_Delay(1);
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
        HAL_Delay(1);

        // reset the gyro sensor
        BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
        HAL_Delay(BMI088_LONG_DELAY_TIME);
        // check commiunication is normal after reset
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
        HAL_Delay(1);
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
        HAL_Delay(1);

        // check the "who am I"
        if (res != BMI088_GYRO_CHIP_ID_VALUE)
            return false;

        // set gyro sonsor config and check
        for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; ++write_reg_num) {
            BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                                         write_BMI088_gyro_reg_data_error[write_reg_num][1]);
            HAL_Delay(1);
            BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], &res);
            HAL_Delay(1);
            if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
                return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }

        return false;
    }

    void BMI088::Read(float* gyro, float* accel, float* temperate) {
        uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
        int16_t bmi088_raw_temp;

        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

        BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;

        *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    void BMI088::temperature_read_over(uint8_t* rx_buf, float* temperate) {
        int16_t bmi088_raw_temp;
        bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    void BMI088::accel_read_over(uint8_t* rx_buf, float* accel, float* time) {
        int16_t bmi088_raw_temp;
        uint32_t sensor_time;
        bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
        accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
        accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
        accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
        sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
        *time = sensor_time * 39.0625f;
    }

    void BMI088::gyro_read_over(uint8_t* rx_buf, float* gyro) {
        int16_t bmi088_raw_temp;
        bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
}