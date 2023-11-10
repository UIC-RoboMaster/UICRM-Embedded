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
#include "bsp_spi.h"
#include "bsp_error_handler.h"

namespace bsp {

    /* get initialized uart_t instance given its hspi handle struct */
    SPI* SPI::FindInstance(SPI_HandleTypeDef* hspi) {
        const auto it = ptr_map.find(hspi);
        if (it == ptr_map.end()) {
            return nullptr;
        }

        return it->second;
    }
    SPI::SPI(spi_init_t init) {
        hspi_ = init.hspi;
        mode_ = init.mode;
        ptr_map[hspi_] = this;
        switch (mode_) {
            case SPI_MODE_BLOCKED:
                break;
            case SPI_MODE_INTURRUPT:
            case SPI_MODE_DMA:
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_RX_COMPLETE_CB_ID, CallbackWrapper);
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_TX_RX_COMPLETE_CB_ID, CallbackWrapper);
        }
    }
    SPI::~SPI() {
        ptr_map.erase(hspi_);
    }

    void SPI::TransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
        rx_size_ = length;
        rx_buffer_ = rx_data;
        switch(mode_){
            case SPI_MODE_BLOCKED:
                HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, length, 1000);
                callback_(this);
                break;
            case SPI_MODE_INTURRUPT:
                HAL_SPI_TransmitReceive_IT(hspi_, tx_data, rx_data, length);
                break;
            case SPI_MODE_DMA:
                HAL_SPI_TransmitReceive_DMA(hspi_, tx_data, rx_data, length);
                break;
            default:
                RM_ASSERT_TRUE(false, "Invalid SPI mode");
        }
    }
    void SPI::Transmit(uint8_t* tx_data, uint32_t length) {
        switch(mode_){
        case SPI_MODE_BLOCKED:
                HAL_SPI_Transmit(hspi_, tx_data, length, 1000);
                break;
        case SPI_MODE_INTURRUPT:
                HAL_SPI_Transmit_IT(hspi_, tx_data, length);
                break;
        case SPI_MODE_DMA:
                HAL_SPI_Transmit_DMA(hspi_, tx_data, length);
                break;
        default:
                RM_ASSERT_TRUE(false, "Invalid SPI mode");
        }
    }
    void SPI::Receive(uint8_t* rx_data, uint32_t length) {
        rx_size_ = length;
        rx_buffer_ = rx_data;
        switch(mode_){
        case SPI_MODE_BLOCKED:
                HAL_SPI_Receive(hspi_, rx_data, length, 1000);
                break;
        case SPI_MODE_INTURRUPT:
                HAL_SPI_Receive_IT(hspi_, rx_data, length);
                break;
        case SPI_MODE_DMA:
                HAL_SPI_Receive_DMA(hspi_, rx_data, length);
                break;
        default:
                RM_ASSERT_TRUE(false, "Invalid SPI mode");
        }
    }

    bool SPI::IsBusy() {
        return HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY;
    }
    void SPI::RegisterCallback(spi_rx_callback_t callback) {
        callback_ = callback;
    }
    void SPI::CallbackWrapper(SPI_HandleTypeDef* hspi) {
        SPI* instance = FindInstance(hspi);
        if (instance == nullptr) {
            return;
        }
        instance->callback_(instance);
    }
    bool SPI::IsDMA() {
        return mode_ == SPI_MODE_DMA;
    }

    SPIMaster::SPIMaster(spi_master_init_t init) {
        spi_ = init.spi;
        cs_ = init.cs;
    }

        SPIMaster::~SPIMaster() {

        }

        spi_master_status_e SPIMaster::Transmit(uint8_t* tx_data, uint32_t length) {
            if (spi_->IsBusy()) {
                return SPI_MASTER_STATUS_BUSY;
            }
            cs_->Low();
            spi_->Transmit(tx_data, length);
            return SPI_MASTER_STATUS_OK;
        }

        spi_master_status_e SPIMaster::Receive(uint8_t* rx_data, uint32_t length) {
            if (spi_->IsBusy()) {
                return SPI_MASTER_STATUS_BUSY;
            }
            cs_->Low();
            spi_->Receive(rx_data, length);
            return SPI_MASTER_STATUS_OK;
        }

        spi_master_status_e SPIMaster::TransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
            if (spi_->IsBusy()) {
                return SPI_MASTER_STATUS_BUSY;
            }
            cs_->Low();
            spi_->TransmitReceive(tx_data, rx_data, length);
            return SPI_MASTER_STATUS_OK;
        }

        void SPIMaster::RegisterCallback(spi_master_rx_callback_t callback) {
            callback_=callback;
        }
}  // namespace bsp

void RM_SPI_IRQHandler(SPI_HandleTypeDef* hspi) {
    bsp::SPI::CallbackWrapper(hspi);
}