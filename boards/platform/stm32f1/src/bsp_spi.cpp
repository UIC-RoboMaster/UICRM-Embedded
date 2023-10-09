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

namespace bsp{



    /* get initialized uart_t instance given its hspi handle struct */
    SPI* SPI::FindInstance(SPI_HandleTypeDef* hspi) {
        const auto it = ptr_map.find(hspi);
        if (it == ptr_map.end()) {
            return nullptr;
        }

        return it->second;
    }
    SPI::SPI(SPI_HandleTypeDef* hspi) {
        hspi_=hspi;
        hdma_spi_tx_ = hspi_->hdmatx;
        hdma_spi_rx_ = hspi_->hdmarx;
        bufsize_=0;
        txbuf_= nullptr;
        rxbuf_= nullptr;
    }
    SPI::~SPI() {

    }
    void SPI::Setup(uint32_t buffer_size, bool dma) {
        dma_=dma;
        bufsize_=buffer_size;
    }
    void SPI::Init() {
        if(dma_){

        }
        else{
            HAL_SPI_Init(hspi_);
        }

    }
    uint32_t SPI::TransimiReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
        if(dma_) {

        }else{
            HAL_SPI_TransmitReceive_IT(hspi_, tx_data, rx_data, length);
        }
        return length;
    }

    bool SPI::IsBusy() {
        return HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY;
    }
    void SPI::RegisterCallback(spi_rx_callback_t callback) {
        callback_=callback;
    }
    void SPI::CallbackWrapper(SPI_HandleTypeDef* hspi) {
        SPI* instance = FindInstance(hspi);
        if (instance == nullptr) {
            return;
        }
        instance->callback_();
    }
    bool SPI::IsDMA() {
        return dma_;
    }
}

void RM_DMA_SPI_IRQHandler(SPI_HandleTypeDef* hspi) {
    bsp::SPI::CallbackWrapper(hspi);
}