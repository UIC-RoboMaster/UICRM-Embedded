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
#include "bsp_imu.h"

#include <cmath>

#include "MahonyAHRS.h"
#include "bsp_error_handler.h"
#include "bsp_mpu6500_reg.h"
#include "bsp_os.h"
#include "dma.h"

#define MPU6500_DELAY 55  // SPI delay
// configured with initialization sequences
#define MPU6500_ACC_FACTOR 4096.0f
#define MPU6500_GYRO_FACTOR 32.768f

#define GRAVITY_ACC 9.8f
#define DEG2RAD(x) ((x) / 180 * M_PI)

#define MAG_SEN 0.3f  // raw int16 data change to uT unit.

#define IST8310_WHO_AM_I 0x00        // ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE 0x10  // device ID

#define IST8310_WRITE_REG_NUM 4

// the first column:the registers of IST8310.
// the second column: the value to be writed to the registers.
// the third column: return error value.
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},   // enalbe interrupt  and low pin polarity.
    {0x41, 0x09, 0x02},   // average 2 times.
    {0x42, 0xC0, 0x03},   // must be 0xC0.
    {0x0A, 0x0B, 0x04}};  // 200Hz output rate.

#define IST8310_IIC_ADDRESS 0x0E  // the I2C address of IST8310

namespace bsp {

    MPU6500* MPU6500::mpu6500 = nullptr;

    MPU6500::MPU6500(SPI_HandleTypeDef* hspi, const GPIO& chip_select, uint16_t int_pin)
        : GPIT(int_pin), hspi_(hspi), chip_select_(chip_select) {
        const uint8_t init_len = 7;
        const uint8_t init_data[init_len][2] = {
            {MPU6500_PWR_MGMT_1, 0x03},      // auto select clock source
            {MPU6500_PWR_MGMT_2, 0x00},      // enable acc & gyro
            {MPU6500_CONFIG, 0x02},          // gyro LP bandwidth 92Hz
            {MPU6500_GYRO_CONFIG, 0x10},     // gyro range 1000 dps / 32.8
            {MPU6500_ACCEL_CONFIG, 0x10},    // acc range 8g / 4096
            {MPU6500_ACCEL_CONFIG_2, 0x02},  // acc LP bandwidth 92Hz
            {MPU6500_INT_PIN_CFG, 0x10},     // any read to clear interrupt
        };
        Reset();  // reset all registers and signal paths
        for (size_t i = 0; i < init_len; ++i)
            WriteReg(init_data[i][0], init_data[i][1]);
        // validate register values
        uint8_t tmp;
        for (size_t i = 0; i < init_len; ++i) {
            ReadReg(init_data[i][0], &tmp);
            if (tmp != init_data[i][1])
                bsp_error_handler(__FUNCTION__, __LINE__, "imu register incorrect initialization");
        }
        // setup interrupt callback
        RM_ASSERT_FALSE(mpu6500, "Repteated intialization of MPU6500");
        mpu6500 = this;
        HAL_SPI_RegisterCallback(hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, &MPU6500::SPITxRxCpltCallback);
        // initialize magnetometer
        IST8310Init();
        // enable imu interrupt
        WriteReg(MPU6500_INT_ENABLE, 0x01);
    }

    void MPU6500::IST8310Init() {
        WriteReg(MPU6500_USER_CTRL, 0x30);     // enable I2C master and reset all slaves
        WriteReg(MPU6500_I2C_MST_CTRL, 0x0d);  // 400 kHz I2C clock
        // slave 0 for auto receive
        WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0e | 0x80);  // read from device 0x0e
        WriteReg(MPU6500_I2C_SLV0_REG, 0x03);          // read data from 0x03 reg
        // slave 1 for auto transmit
        WriteReg(MPU6500_I2C_SLV1_ADDR, 0x0e);  // write into device 0x0e
        WriteReg(MPU6500_I2C_SLV1_REG, 0x0a);   // write data into 0x0a reg
        WriteReg(MPU6500_I2C_SLV1_DO, 0x01);    // send measurement command
        // enable slave 0 and 1
        WriteReg(MPU6500_I2C_SLV0_CTRL, 0xd6);  // swap endian + 6 bytes rx
        WriteReg(MPU6500_I2C_SLV1_CTRL, 0x81);  // 1 bytes tx
    }

    void MPU6500::UpdateData() {
        chip_select_.Low();
        io_buff_[0] = MPU6500_ACCEL_XOUT_H | 0x80;
        HAL_SPI_TransmitReceive_DMA(hspi_, io_buff_, io_buff_, MPU6500_SIZEOF_DATA + 1);
    }

    void MPU6500::Reset() {
        WriteReg(MPU6500_PWR_MGMT_1, 0x80);
        WriteReg(MPU6500_SIGNAL_PATH_RESET, 0x07);
        WriteReg(MPU6500_USER_CTRL, 0x03);

        HAL_Delay(1);  // seems like signal path reset needs some time
    }

    void MPU6500::WriteReg(uint8_t reg, uint8_t data) {
        WriteRegs(reg, &data, 1);
    }

    void MPU6500::WriteRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
        uint8_t tx = reg_start & 0x7f;

        chip_select_.Low();
        HAL_SPI_Transmit(hspi_, &tx, 1, MPU6500_DELAY);
        HAL_SPI_Transmit(hspi_, data, len, MPU6500_DELAY);
        chip_select_.High();
    }

    void MPU6500::ReadReg(uint8_t reg, uint8_t* data) {
        ReadRegs(reg, data, 1);
    }

    void MPU6500::ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
        chip_select_.Low();
        *data = static_cast<uint8_t>(reg_start | 0x80);
        HAL_SPI_Transmit(hspi_, data, 1, MPU6500_DELAY);
        HAL_SPI_Receive(hspi_, data, len, MPU6500_DELAY);
        chip_select_.High();
    }

    void MPU6500::SPITxRxCpltCallback() {
        chip_select_.High();
        // NOTE(alvin): per MPU6500 documentation, the first byte of the rx / tx
        // buffer
        //              contains the address of the SPI device
        uint8_t* buff = io_buff_ + 1;
        int16_t* array = reinterpret_cast<int16_t*>(buff);
        // in-place swap endian
        for (size_t i = 0; i < MPU6500_SIZEOF_DATA; i += 2)
            array[i / 2] = (int16_t)(buff[i] << 8 | buff[i + 1]);

        acce.x = (float)array[0] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
        acce.y = (float)array[1] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
        acce.z = (float)array[2] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
        temp = (float)array[3] / MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;
        gyro.x = DEG2RAD((float)array[4] / MPU6500_GYRO_FACTOR);
        gyro.y = DEG2RAD((float)array[5] / MPU6500_GYRO_FACTOR);
        gyro.z = DEG2RAD((float)array[6] / MPU6500_GYRO_FACTOR);
        mag.x = (float)array[7];
        mag.y = (float)array[8];
        mag.z = (float)array[9];
    }

    void MPU6500::IntCallback() {
        timestamp = GetHighresTickMicroSec();
        UpdateData();
    }

    void MPU6500::SPITxRxCpltCallback(SPI_HandleTypeDef* hspi) {
        UNUSED(hspi);
        mpu6500->SPITxRxCpltCallback();
    }

    IST8310::IST8310(IST8310_init_t init) : GPIT(init.int_pin) {
        hi2c_ = init.hi2c;
        rst_group_ = init.rst_group;
        rst_pin_ = init.rst_pin;
        Init();
    }

    IST8310::IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, GPIO_TypeDef* rst_group,
                     uint16_t rst_pin)
        : GPIT(int_pin) {
        hi2c_ = hi2c;
        rst_group_ = rst_group;
        rst_pin_ = rst_pin;
        Init();
    }

    bool IST8310::IsReady() {
        return HAL_I2C_IsDeviceReady(hi2c_, IST8310_IIC_ADDRESS << 1, 1, 100) == HAL_OK;
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
    }

    void IST8310::ist8310_RST_H() {
        HAL_GPIO_WritePin(rst_group_, rst_pin_, GPIO_PIN_SET);
    }

    void IST8310::ist8310_RST_L() {
        HAL_GPIO_WritePin(rst_group_, rst_pin_, GPIO_PIN_RESET);
    }

    uint8_t IST8310::ist8310_IIC_read_single_reg(uint8_t reg) {
        uint8_t res = 0;
        HAL_I2C_Mem_Read(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10);
        return res;
    }

    void IST8310::ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
        HAL_I2C_Mem_Write(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    }

    void IST8310::ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
        HAL_I2C_Mem_Read(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
    }

    void IST8310::ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t* data, uint8_t len) {
        HAL_I2C_Mem_Write(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len,
                          10);
    }
}