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

#include "wit_protocol.h"
#include "arm_math.h"


namespace imu {
    WITUART::WITUART(bsp::UART* uart) {
        uart_=uart;
    }


    void WITUART::Update(bool fromISR) {
        uint8_t* data;
        int length;
        if(fromISR)
            length=uart_->Read<true>(&data);
        else
            length=uart_->Read<false>(&data);
        if(length<11){
            return;
        }
        if(data[0]!=0x55){
            return;
        }
        uint8_t checksum=0;
        for(int i=0;i<10;i++){
            checksum+=data[i];
        }
        if(checksum!=data[10]){
            return;
        }
        if(data[1]==WIT_RX_ACCEL) {
            accel_[0] = (float)((int16_t)((int16_t)data[3] << 8 | data[2])) / 32768.0f * 16;
            accel_[1] = (float)((int16_t)((int16_t)data[5] << 8 | data[4])) / 32768.0f * 16;
            accel_[2] = (float)((int16_t)((int16_t)data[7] << 8 | data[6])) / 32768.0f * 16;
            temp_ = (float)((int16_t)((int16_t)data[9] << 8 | data[8])) / 100.0f;
        }
        else if(data[1]==WIT_RX_GYRO) {
            gyro_[0] = (float)((int16_t)((int16_t)data[3] << 8 | data[2])) / 32768.0f * 2000;
            gyro_[1] = (float)((int16_t)((int16_t)data[5] << 8 | data[4])) / 32768.0f * 2000;
            gyro_[2] = (float)((int16_t)((int16_t)data[7] << 8 | data[6])) / 32768.0f * 2000;
        } else if(data[1]==WIT_RX_INS) {
                INS_angle[0]=(float)((int16_t)((int16_t)data[3]<<8|data[2]))/32768.0f*PI;
                INS_angle[1]=(float)((int16_t)((int16_t)data[5]<<8|data[4]))/32768.0f*PI;
                INS_angle[2]=(float)((int16_t)((int16_t)data[7]<<8|data[6]))/32768.0f*PI;
                version_=(int16_t)data[9]<<8|data[8];
        } else if(data[1]==WIT_RX_MAG) {
                mag_[0]=((int16_t)((int16_t)data[3]<<8|data[2]));
                mag_[1]=((int16_t)((int16_t)data[5]<<8|data[4]));
                mag_[2]=((int16_t)((int16_t)data[7]<<8|data[6]));
                temp_=(float)((int16_t)((int16_t)data[9]<<8|data[8]))/100.0f;
        } else if(data[1]==WIT_RX_QUAT) {
                quat_[0]=(float)((int16_t)((int16_t)data[3]<<8|data[2]))/32768.0f;
                quat_[1]=(float)((int16_t)((int16_t)data[5]<<8|data[4]))/32768.0f;
                quat_[2]=(float)((int16_t)((int16_t)data[7]<<8|data[6]))/32768.0f;
                quat_[3]=(float)((int16_t)((int16_t)data[9]<<8|data[8]))/32768.0f;
        } else if(data[1]==WIT_RX_PORT_STATUS) {
                port_status_[0]=(int16_t)data[3]<<8|data[2];
                port_status_[1]=(int16_t)data[5]<<8|data[4];
                port_status_[2]=(int16_t)data[7]<<8|data[6];
                port_status_[3]=(int16_t)data[9]<<8|data[8];
        } else if(data[1]==WIT_RX_READ_REG) {
                read_reg_data_[0]=(int16_t)data[3]<<8|data[2];
                read_reg_data_[1]=(int16_t)data[5]<<8|data[4];
                read_reg_data_[2]=(int16_t)data[7]<<8|data[6];
                read_reg_data_[3]=(int16_t)data[9]<<8|data[8];
                read_callback_();
        }


    }
    void WITUART::Unlock() {
        uint8_t send_data[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
        uart_->Write(send_data, 5);
    }
    void WITUART::Lock() {
        uint8_t send_data[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
        uart_->Write(send_data, 5);
    }
    void WITUART::WriteReg(uint8_t reg, uint8_t* data) {
        uint8_t send_data[5] = {0xFF, 0xAA, reg, data[0], data[1]};
        uart_->Write(send_data, 5);
    }
    void WITUART::ReadReg(uint8_t reg, uint8_t func,uint16_t* data) {
        uint8_t send_data[5] = {0xFF, 0xAA, 0x27,reg, func};
        read_reg_data_ = data;
        uart_->Write(send_data, 5);
    }
    void WITUART::RegisterReadCallback(wit_read_callback_t callback) {
        read_callback_ = callback;
    }

}