/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "MPU6500.h"

#include "bsp_os.h"

namespace imu {
    MPU6500::MPU6500(mpu6500_init_t init) {
        spi_ = init.spi;
        cs_ = init.cs;
        spi_device_ = spi_->NewDevice(cs_);
        int_pin_ = init.int_pin;
        spi_->SetAutoCS(false);
        spi_->SetMode(bsp::SPI_MODE_BLOCKED);
        dma_ = init.dma;
        use_mag_ = init.use_mag;
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
        spi_device_->RegisterCallback(SPITxRxCpltCallbackWrapper, this);
        // initialize magnetometer
        if(use_mag_)
            IST8310Init();
        // enable imu interrupt
        WriteReg(MPU6500_INT_ENABLE, 0x01);
        if (dma_) {
            spi_->SetMode(bsp::SPI_MODE_DMA);
        } else {
            spi_->SetMode(bsp::SPI_MODE_INTURRUPT);
        }

        bsp::thread_init_t thread_init = {
            .func = callback_thread_func_,
            .args = this,
            .attr = callback_thread_attr_,
        };
        callback_thread_ = new bsp::EventThread(thread_init);

        int_pin_->RegisterCallback(IntCallback, this);
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
        spi_device_->PrepareTransmit();
        io_buff_[0] = MPU6500_ACCEL_XOUT_H | 0x80;
        if(use_mag_)
            spi_->TransmitReceive(spi_device_, io_buff_, io_buff_, MPU6500_SIZEOF_DATA + 1);
        else
            spi_->TransmitReceive(spi_device_, io_buff_, io_buff_, MPU6500_SIZEOF_DATA+1-6);
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

        spi_device_->PrepareTransmit();
        spi_->Transmit(spi_device_, &tx, 1);
        spi_->Transmit(spi_device_, data, len);
        spi_device_->FinishTransmit();
    }

    void MPU6500::ReadReg(uint8_t reg, uint8_t* data) {
        ReadRegs(reg, data, 1);
    }

    void MPU6500::ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
        spi_device_->PrepareTransmit();
        *data = static_cast<uint8_t>(reg_start | 0x80);
        spi_->Transmit(spi_device_, data, 1);
        spi_->Receive(spi_device_, data, len);
        spi_device_->FinishTransmit();
    }

    void MPU6500::SPITxRxCpltCallback() {
        // NOTE(alvin): per MPU6500 documentation, the first byte of the rx / tx
        // buffer
        //              contains the address of the SPI device
        uint8_t* buff = io_buff_ + 1;
        int16_t* array = reinterpret_cast<int16_t*>(buff);
        // in-place swap endian
        for (size_t i = 0; i < MPU6500_SIZEOF_DATA; i += 2)
            array[i / 2] = (int16_t)(buff[i] << 8 | buff[i + 1]);

        accel_[0] = (float)array[0] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
        accel_[1] = (float)array[1] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
        accel_[2] = (float)array[2] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
        temperature_ = (float)array[3] / MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;
        gyro_[0] = DEG2RAD((float)array[4] / MPU6500_GYRO_FACTOR);
        gyro_[1] = DEG2RAD((float)array[5] / MPU6500_GYRO_FACTOR);
        gyro_[2] = DEG2RAD((float)array[6] / MPU6500_GYRO_FACTOR);
        if(use_mag_){
            mag_[0] = (float)array[7];
            mag_[1] = (float)array[8];
            mag_[2] = (float)array[9];
        }
        if (callback_thread_ != nullptr)
            callback_thread_->Set();
        spi_device_->FinishTransmit();
    }

    void MPU6500::IntCallback(void* args) {
        if (args == nullptr)
            return;
        MPU6500* mpu6500 = reinterpret_cast<MPU6500*>(args);
        mpu6500->time_ = (float)bsp::GetHighresTickMicroSec();
        mpu6500->UpdateData();
    }

    void MPU6500::SPITxRxCpltCallbackWrapper(void* args) {
        if (args == nullptr)
            return;
        MPU6500* mpu6500 = reinterpret_cast<MPU6500*>(args);
        mpu6500->SPITxRxCpltCallback();
    }
    void MPU6500::RegisterCallback(mpu6500_callback_t callback) {
        callback_ = callback;
    }

    void MPU6500::callback_thread_func_(void* arg) {
        MPU6500* mpu6500 = reinterpret_cast<MPU6500*>(arg);
        if(mpu6500->callback_ != nullptr)
            mpu6500->callback_();
    }
    MPU6500::~MPU6500() {
        delete callback_thread_;
    }
}  // namespace imu