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

#include "bsp_i2c.h"

#include <cstring>

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace bsp {

    std::map<I2C_HandleTypeDef*, I2C*> I2C::ptr_map;

    /**
     * @brief find instantiated can line
     *
     * @param hcan  HAL can handle
     *
     * @return can instance if found, otherwise NULL
     */
    I2C* I2C::FindInstance(I2C_HandleTypeDef* hi2c) {
        const auto it = ptr_map.find(hi2c);
        if (it == ptr_map.end())
            return nullptr;

        return it->second;
    }

    /**
     * @brief check if any associated CAN instance is instantiated or not
     *
     * @param hcan  HAL can handle
     *
     * @return true if found, otherwise false
     */
    bool I2C::HandleExists(I2C_HandleTypeDef* hi2c) {
        return FindInstance(hi2c) != nullptr;
    }

    /**
     * @brief callback handler for CAN rx feedback data
     *
     * @param hcan  HAL can handle
     */
    void I2C::I2CRxCallback(I2C_HandleTypeDef* hi2c) {
        I2C* i2c = FindInstance(hi2c);
        if (!i2c)
            return;
        i2c->RxCallback();
    }

    I2C::I2C(I2C_HandleTypeDef* hi2c, bool is_master, bool is_dma)
        : hi2c_(hi2c) {

        is_master_ = is_master;
        is_dma_ = is_dma;
        // save can instance as global pointer
        ptr_map[hi2c] = this;
        // HAL_I2C_RegisterCallback(hi2c_, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, I2CRxCallback);
    }

    bool I2C::isReady(uint16_t id, uint32_t timeout) {
        return HAL_I2C_IsDeviceReady(hi2c_, id, 1, timeout) == HAL_OK;
    }

    int I2C::RegisterRxCallback(uint32_t std_id, i2c_rx_callback_t callback, void* args) {
        // int callback_id = std_id - start_id_;
        // todo: Rx Callback
        if (callback_count_ >= MAX_I2C_DEVICES)
            return -1;

        rx_args_[callback_count_] = args;
        rx_callbacks_[callback_count_] = callback;
        id_to_index_[std_id] = callback_count_;
        callback_count_++;

        return 0;
    }

    int I2C::Transmit(uint16_t id, const uint8_t data[], uint16_t length) {
        if(HAL_I2C_Master_Transmit(hi2c_, id, const_cast<uint8_t*>(data), length, 20)!= HAL_OK){
            return -1;
        }
        return length;
    }

    int I2C::Receive(uint16_t id, uint8_t* data, uint16_t length) {
        if(HAL_I2C_Master_Receive(hi2c_, id, const_cast<uint8_t*>(data), length, 50)!= HAL_OK){
            return -1;
        }
        return length;
    }

    int I2C::MemoryRead(uint16_t id, uint16_t reg, uint8_t* data, uint16_t length){
        if(HAL_I2C_Mem_Read(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length, 20) != HAL_OK){
            return -1;
        }
        return length;
    }

    int I2C::MemoryRead(uint16_t id, uint16_t reg, uint16_t* data, uint16_t length){
        uint8_t *pData;
        pData = (uint8_t*)data;
        if(HAL_I2C_Mem_Read(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length, 20) != HAL_OK){
            return -1;
        }
        return length;
    }

    int I2C::MemoryWrite(uint16_t id, uint16_t reg, uint8_t* data, uint16_t length){
        if(HAL_I2C_Mem_Write(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length, 20) != HAL_OK){
            return -1;
        }
        return length;
    }

    int I2C::MemoryWrite(uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
        uint8_t *pData;
        pData = (uint8_t*)data;
        if(HAL_I2C_Mem_Write(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length, 20) != HAL_OK){
            return -1;
        }
        return length;
    }



    void I2C::RxCallback() {
        //TODO: I2C Rx callback
        uint8_t data[MAX_I2C_DATA_SIZE];
        memcpy(data, rx_data_, MAX_I2C_DATA_SIZE);

        uint16_t callback_id = 0;
        const auto it = id_to_index_.find(callback_id);
        if (it == id_to_index_.end())
            return;
        callback_id = it->second;
        // find corresponding callback
        if (rx_callbacks_[callback_id])
            rx_callbacks_[callback_id](data, rx_args_[callback_id]);

        HAL_I2C_Master_Receive_DMA(hi2c_, 0x00, rx_data_, MAX_I2C_DATA_SIZE);
    }



} /* namespace bsp */