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

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    bsp::I2C::CallbackWrapper(hi2c);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    bsp::I2C::CallbackWrapper(hi2c);
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    bsp::I2C::CallbackWrapper(hi2c);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    bsp::I2C::CallbackWrapper(hi2c);
}

namespace bsp {

    std::unordered_map<I2C_HandleTypeDef*, I2C*> I2C::ptr_map;

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

    I2C::I2C(i2c_init_t init) : hi2c_(init.hi2c), mode_(init.mode) {
        // save can instance as global pointer
        ptr_map[hi2c_] = this;
    }

    bool I2C::isReady(uint16_t id, uint32_t timeout) {
        return HAL_I2C_IsDeviceReady(hi2c_, id, 1, timeout) == HAL_OK;
    }

    int I2C::RegisterRxCallback(uint32_t std_id, i2c_rx_callback_t callback, void* args){
        // int callback_id = std_id - start_id_;
        // todo: Rx Callback
        if (callback_count_ >= MAX_I2C_DEVICES)
            return -1;

        rx_callbacks_[callback_count_] = callback;
        id_to_index_[std_id] = callback_count_;
        rx_args_[callback_count_] = args;
        callback_count_++;

        return 0;
    }

    int I2C::Transmit(uint16_t id, const uint8_t data[], uint16_t length) {
        switch (mode_) {
            case I2C_MODE_BLOCKING:
                return HAL_I2C_Master_Transmit(hi2c_, id, const_cast<uint8_t*>(data), length, 20);
                break;
            case I2C_MODE_IT:
                return HAL_I2C_Master_Transmit_IT(hi2c_, id, const_cast<uint8_t*>(data), length);
                break;
            case I2C_MODE_DMA:
                return HAL_I2C_Master_Transmit_DMA(hi2c_, id, const_cast<uint8_t*>(data), length);
                break;
        }
        return 0;
    }

    int I2C::Receive(uint16_t id, uint8_t* data, uint16_t length) {
        switch (mode_) {
            case I2C_MODE_BLOCKING:
                return HAL_I2C_Master_Receive(hi2c_, id, data, length, 20);
                break;
            case I2C_MODE_IT:
                return HAL_I2C_Master_Receive_IT(hi2c_, id, data, length);
                break;
            case I2C_MODE_DMA:
                return HAL_I2C_Master_Receive_DMA(hi2c_, id, data, length);
                break;
        }
        return 0;
    }

    int I2C::MemoryRead(uint16_t id, uint16_t reg, uint8_t* data, uint16_t length) {
        switch (mode_) {
            case I2C_MODE_BLOCKING:
                return HAL_I2C_Mem_Read(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length, 20);
                break;
            case I2C_MODE_IT:
                return HAL_I2C_Mem_Read_IT(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length);
                break;
            case I2C_MODE_DMA:
                return HAL_I2C_Mem_Read_DMA(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length);
        }
        return 0;
    }

    int I2C::MemoryRead(uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
        uint8_t* pData;
        pData = (uint8_t*)data;
        switch (mode_) {
            case I2C_MODE_BLOCKING:
                return HAL_I2C_Mem_Read(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length, 20);
                break;
            case I2C_MODE_IT:
                return HAL_I2C_Mem_Read_IT(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length);
                break;
            case I2C_MODE_DMA:
                return HAL_I2C_Mem_Read_DMA(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length);
        }
        return 0;
    }

    int I2C::MemoryWrite(uint16_t id, uint16_t reg, uint8_t* data, uint16_t length) {
        switch (mode_) {
            case I2C_MODE_BLOCKING:
                return HAL_I2C_Mem_Write(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length, 20);
                break;
            case I2C_MODE_IT:
                return HAL_I2C_Mem_Write_IT(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length);
                break;
            case I2C_MODE_DMA:
                return HAL_I2C_Mem_Write_DMA(hi2c_, id, reg, I2C_MEMADD_SIZE_8BIT, data, length);
                break;
        }
        return 0;
    }

    int I2C::MemoryWrite(uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
        uint8_t* pData;
        pData = (uint8_t*)data;
        switch (mode_) {
            case I2C_MODE_BLOCKING:
                return HAL_I2C_Mem_Write(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length, 20);
                break;
            case I2C_MODE_IT:
                return HAL_I2C_Mem_Write_IT(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length);
                break;
            case I2C_MODE_DMA:
                return HAL_I2C_Mem_Write_DMA(hi2c_, id, reg, I2C_MEMADD_SIZE_16BIT, pData, length);
                break;
        }
        return 0;
    }

    void I2C::CallbackWrapper(I2C_HandleTypeDef* hi2c) {
        I2C* i2c = FindInstance(hi2c);
        if (!i2c)
            return;
        if (i2c->mode_ == I2C_MODE_BLOCKING)
            return;
        i2c->RxCallback();
    }

    void I2C::RxCallback() {
        uint16_t callback_id = hi2c_->Devaddress;
        const auto it = id_to_index_.find(callback_id);
        if (it == id_to_index_.end())
            return;
        callback_id = it->second;
        // find corresponding callback
        if (rx_callbacks_[callback_id])
            rx_callbacks_[callback_id](rx_args_[callback_id]);
    }
    void I2C::SetMode(i2c_mode_e mode) {
        mode_ = mode;
    }
    i2c_mode_e I2C::GetMode() {
        return mode_;
    }

} /* namespace bsp */