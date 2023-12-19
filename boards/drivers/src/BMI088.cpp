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

#include <string.h>

namespace imu {


    BMI088::BMI088(BMI088_init_t init) {
        spi_master_ = init.spi_master;
        spi_device_accel_ = spi_master_->NewDevice(init.CS_ACCEL);
        spi_device_gyro_ = spi_master_->NewDevice(init.CS_GYRO);
        gpit_accel_ = init.INT_ACCEL;
        gpit_gyro_ = init.INT_GYRO;
        dma_ = init.is_DMA;
        while (Init() != BMI088_NO_ERROR)
            ;
    }

    BMI088::BMI088(bsp::SPIMaster* spi_master, bsp::GPIO* CS_ACCEL, bsp::GPIO* CS_GYRO,
                   bsp::GPIT* INT_ACCEL, bsp::GPIT* INT_GYRO, bool is_DMA) {
        spi_master_ = spi_master;
        spi_device_accel_ = spi_master_->NewDevice(CS_ACCEL);
        spi_device_gyro_ = spi_master_->NewDevice(CS_GYRO);
        gpit_accel_ = INT_ACCEL;
        gpit_gyro_ = INT_GYRO;
        dma_ = is_DMA;
        while (Init() != BMI088_NO_ERROR)
            ;
    }

    bool BMI088::IsReady() {
        return bmi088_start_flag;
    }

    void BMI088::BMI088_accel_write_single_reg(uint8_t reg, uint8_t data) {
        uint8_t rx_data;
        spi_device_accel_->PrepareTransmit();
        spi_master_->TransmitReceive(spi_device_accel_, &reg, &rx_data, 1);
        spi_master_->TransmitReceive(spi_device_accel_, &data, &rx_data, 1);
        spi_device_accel_->FinishTransmit();
    }

    void BMI088::BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data) {
        uint8_t rx_data;
        uint8_t tx_data = reg | 0x80;
        spi_device_accel_->PrepareTransmit();
        spi_master_->TransmitReceive(spi_device_accel_, &tx_data, &rx_data, 1);
        tx_data = 0x55;
        spi_master_->TransmitReceive(spi_device_accel_, &tx_data, &rx_data, 1);
        spi_master_->TransmitReceive(spi_device_accel_, &tx_data, data, 1);
        spi_device_accel_->FinishTransmit();
    }

    void BMI088::BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        spi_device_accel_->PrepareTransmit();
        uint8_t rx_data;
        uint8_t tx_data = reg | 0x80;
        spi_master_->TransmitReceive(spi_device_accel_, &tx_data, &rx_data, 1);

        spi_master_->TransmitReceive(spi_device_accel_, &tx_data, &rx_data, 1);
        uint8_t* tmp_ptr = buf;
        tx_data = 0x55;
        while (len != 0) {
            spi_master_->TransmitReceive(spi_device_accel_, &tx_data, tmp_ptr, 1);
            tmp_ptr++;
            len--;
        }
        spi_device_accel_->FinishTransmit();
    }

    void BMI088::BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
        uint8_t rx_data;
        spi_device_gyro_->PrepareTransmit();
        spi_master_->TransmitReceive(spi_device_gyro_, &reg, &rx_data, 1);
        spi_master_->TransmitReceive(spi_device_gyro_, &data, &rx_data, 1);
        spi_device_gyro_->FinishTransmit();
    }

    void BMI088::BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data) {
        uint8_t rx_data;
        uint8_t tx_data = reg | 0x80;
        spi_device_gyro_->PrepareTransmit();
        spi_master_->TransmitReceive(spi_device_gyro_, &tx_data, &rx_data, 1);
        tx_data = 0x55;
        spi_master_->TransmitReceive(spi_device_gyro_, &tx_data, data, 1);
        spi_device_gyro_->FinishTransmit();
    }

    void BMI088::BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        spi_device_gyro_->PrepareTransmit();
        uint8_t rx_data;
        uint8_t tx_data = reg | 0x80;
        spi_master_->TransmitReceive(spi_device_gyro_, &tx_data, &rx_data, 1);
        uint8_t* tmp_ptr = buf;
        tx_data = 0x55;
        while (len != 0) {
            spi_master_->TransmitReceive(spi_device_gyro_, &tx_data, tmp_ptr, 1);
            tmp_ptr++;
            len--;
        }
        spi_device_gyro_->FinishTransmit();
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

        gpit_accel_->RegisterCallback(BMI088::AccelCallbackWrapper,this);
        gpit_gyro_->RegisterCallback(BMI088::GyroCallbackWrapper,this);

        spi_master_->SetMode(bsp::SPI_MODE_BLOCKED);
        spi_master_->SetAutoCS(false);
        error |= bmi088_accel_init();
        error |= bmi088_gyro_init();

        if (error != BMI088_NO_ERROR) {
            RM_ASSERT_TRUE(false, "BMI088 init error");
        }

        Read(this->gyro_, this->accel_, &(this->temperature_));

        // spi_->hspi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        spi_device_accel_->RegisterCallback(BMI088::AccelSPICallbackWrapper,this);
        spi_device_gyro_->RegisterCallback(BMI088::GyroSPICallbackWrapper,this);
        if (dma_) {
            spi_master_->SetMode(bsp::SPI_MODE_DMA);
        } else {
            spi_master_->SetMode(bsp::SPI_MODE_INTURRUPT);
        }

        bmi088_start_flag = 1;

        return error;
    }

    uint8_t BMI088::bmi088_accel_init() {
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
        res = 0;
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
        HAL_Delay(1);
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
        HAL_Delay(1);

        // check the "who am I"
        if (res != BMI088_ACC_CHIP_ID_VALUE)
            return BMI088_SELF_TEST_ACCEL_ERROR;

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
        return BMI088_NO_ERROR;
    }

    uint8_t BMI088::bmi088_gyro_init() {
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
        res = 0;
        // check commiunication is normal after reset
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
        HAL_Delay(1);
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
        HAL_Delay(1);

        // check the "who am I"
        if (res != BMI088_GYRO_CHIP_ID_VALUE)
            return BMI088_SELF_TEST_GYRO_ERROR;

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

        return BMI088_NO_ERROR;
    }

    void BMI088::Read(float* gyro, float* accel, float* temperate) {
        // Read the data in blocking mode
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

    void BMI088::Read_IT() {
        if (gyro_update_flag & (1 << BMI088_IMU_NOTIFY_SHFITS)) {
            gyro_update_flag &= ~(1 << BMI088_IMU_NOTIFY_SHFITS);
            gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, gyro_);
        }

        if (accel_update_flag & (1 << BMI088_IMU_UPDATE_SHFITS)) {
            accel_update_flag &= ~(1 << BMI088_IMU_UPDATE_SHFITS);
            accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, accel_, &time_);
        }

        if (accel_temp_update_flag & (1 << BMI088_IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << BMI088_IMU_UPDATE_SHFITS);
            temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                  &temperature_);
        }
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

    void BMI088::imu_cmd_spi() {
        if ((gyro_update_flag & (1 << BMI088_IMU_DR_SHFITS)) && !spi_master_->IsBusy() &&
            !(accel_update_flag & (1 << BMI088_IMU_SPI_SHFITS)) &&
            !(accel_temp_update_flag & (1 << BMI088_IMU_SPI_SHFITS))) {
            gyro_update_flag &= ~(1 << BMI088_IMU_DR_SHFITS);
            gyro_update_flag |= (1 << BMI088_IMU_SPI_SHFITS);
            spi_device_gyro_->PrepareTransmit();
            spi_master_->TransmitReceive(spi_device_gyro_, gyro_dma_tx_buf, gyro_dma_rx_buf,
                                         BMI088_SPI_DMA_GYRO_LENGHT);
            return;
        }

        if ((accel_update_flag & (1 << BMI088_IMU_DR_SHFITS)) && !spi_master_->IsBusy() &&
            !(gyro_update_flag & (1 << BMI088_IMU_SPI_SHFITS)) &&
            !(accel_temp_update_flag & (1 << BMI088_IMU_SPI_SHFITS))) {
            accel_update_flag &= ~(1 << BMI088_IMU_DR_SHFITS);
            accel_update_flag |= (1 << BMI088_IMU_SPI_SHFITS);

            spi_device_accel_->PrepareTransmit();
            spi_master_->TransmitReceive(spi_device_accel_, accel_dma_tx_buf, accel_dma_rx_buf,
                                         BMI088_SPI_DMA_ACCEL_LENGHT);
            return;
        }
        if ((accel_temp_update_flag & (1 << BMI088_IMU_DR_SHFITS)) && !spi_master_->IsBusy() &&
            !(gyro_update_flag & (1 << BMI088_IMU_SPI_SHFITS)) &&
            !(accel_update_flag & (1 << BMI088_IMU_SPI_SHFITS))) {
            accel_temp_update_flag &= ~(1 << BMI088_IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << BMI088_IMU_SPI_SHFITS);

            spi_device_accel_->PrepareTransmit();
            spi_master_->TransmitReceive(spi_device_accel_, accel_temp_dma_tx_buf,
                                         accel_temp_dma_rx_buf, BMI088_SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
    }

    void BMI088::AccelSPICallbackWrapper(void* args) {
        if (args == nullptr)
            return;
        BMI088* bmi088 = reinterpret_cast<BMI088*>(args);
        if (bmi088->accel_update_flag & (1 << BMI088_IMU_SPI_SHFITS)) {
            bmi088->accel_update_flag &= ~(1 << BMI088_IMU_SPI_SHFITS);
            bmi088->accel_update_flag |= (1 << BMI088_IMU_UPDATE_SHFITS);
            bmi088->spi_device_accel_->FinishTransmit();
        }
        // temperature read over
        if (bmi088->accel_temp_update_flag & (1 << BMI088_IMU_SPI_SHFITS)) {
            bmi088->accel_temp_update_flag &= ~(1 << BMI088_IMU_SPI_SHFITS);
            bmi088->accel_temp_update_flag |= (1 << BMI088_IMU_UPDATE_SHFITS);
            bmi088->spi_device_accel_->FinishTransmit();
        }
        bmi088->imu_cmd_spi();
    }

    void BMI088::GyroSPICallbackWrapper(void* args) {
        if (args == nullptr)
            return;
        BMI088* bmi088 = reinterpret_cast<BMI088*>(args);
        if (bmi088->gyro_update_flag & (1 << BMI088_IMU_SPI_SHFITS)) {
            bmi088->gyro_update_flag &= ~(1 << BMI088_IMU_SPI_SHFITS);
            bmi088->gyro_update_flag |= (1 << BMI088_IMU_UPDATE_SHFITS);
            bmi088->spi_device_gyro_->FinishTransmit();
        }
        bmi088->imu_cmd_spi();

        if (bmi088->gyro_update_flag & (1 << BMI088_IMU_UPDATE_SHFITS)) {
            bmi088->gyro_update_flag &= ~(1 << BMI088_IMU_UPDATE_SHFITS);
            bmi088->gyro_update_flag |= (1 << BMI088_IMU_NOTIFY_SHFITS);
            bmi088->Read_IT();
            bmi088->RxCompleteCallback();
        }
    }
    void BMI088::GyroCallbackWrapper(void* args) {
        if (args == nullptr)
            return;
        BMI088* bmi088 = reinterpret_cast<BMI088*>(args);
        bmi088->gyro_update_flag |= 1 << BMI088_IMU_DR_SHFITS;
        if (bmi088->bmi088_start_flag)
            bmi088->imu_cmd_spi();
    }
    void BMI088::AccelCallbackWrapper(void* args) {
        if (args == nullptr)
            return;
        BMI088* bmi088 = reinterpret_cast<BMI088*>(args);
        bmi088->accel_update_flag |= 1 << BMI088_IMU_DR_SHFITS;
        bmi088->accel_temp_update_flag |= 1 << BMI088_IMU_DR_SHFITS;
        if (bmi088->bmi088_start_flag)
            bmi088->imu_cmd_spi();
    }
    void BMI088::RegisterCallback(BMI088_callback_t callback) {
        callback_ = callback;
    }

    void BMI088::RxCompleteCallback() {
        callback_();
    }

}  // namespace imu