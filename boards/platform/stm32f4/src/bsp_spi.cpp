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

namespace bsp {

    /* get initialized uart_t instance given its hspi handle struct */
    SPI* SPI::FindInstance(SPI_HandleTypeDef* hspi) {
        const auto it = ptr_map.find(hspi);
        if (it == ptr_map.end()) {
            return nullptr;
        }

        return it->second;
    }
    SPI::SPI(SPI_HandleTypeDef* hspi) {
        hspi_ = hspi;
        hdma_spi_tx_ = hspi_->hdmatx;
        hdma_spi_rx_ = hspi_->hdmarx;
        bufsize_ = 0;
        txbuf_ = nullptr;
        rxbuf_ = nullptr;
    }
    SPI::~SPI() {
    }
    void SPI::Setup(uint32_t buffer_size, bool dma) {
        dma_ = dma;
        bufsize_ = buffer_size;
    }
    void SPI::Init() {
        if (dma_ && hdma_spi_tx_ != nullptr && hdma_spi_rx_ != nullptr) {
            SET_BIT(hspi_->Instance->CR2, SPI_CR2_TXDMAEN);
            SET_BIT(hspi_->Instance->CR2, SPI_CR2_RXDMAEN);

            __HAL_SPI_ENABLE(hspi_);

            // disable DMA
            __HAL_DMA_DISABLE(hdma_spi_rx_);

            while (hdma_spi_rx_->Instance->CR & DMA_SxCR_EN) {
                __HAL_DMA_DISABLE(hdma_spi_rx_);
            }

            __HAL_DMA_CLEAR_FLAG(hdma_spi_rx_, DMA_LISR_TCIF2);

            hdma_spi_rx_->Instance->PAR = (uint32_t) & (SPI1->DR);
            // memory buffer 1
            hdma_spi_rx_->Instance->M0AR = (uint32_t)(rxbuf_);
            // data length
            __HAL_DMA_SET_COUNTER(hdma_spi_rx_, bufsize_);

            __HAL_DMA_ENABLE_IT(hdma_spi_rx_, DMA_IT_TC);

            // disable DMA
            __HAL_DMA_DISABLE(hdma_spi_tx_);

            while (hdma_spi_tx_->Instance->CR & DMA_SxCR_EN) {
                __HAL_DMA_DISABLE(hdma_spi_tx_);
            }

            __HAL_DMA_CLEAR_FLAG(hdma_spi_tx_, DMA_LISR_TCIF3);

            hdma_spi_tx_->Instance->PAR = (uint32_t) & (SPI1->DR);
            // memory buffer 1
            hdma_spi_tx_->Instance->M0AR = (uint32_t)(txbuf_);
            // data length
            __HAL_DMA_SET_COUNTER(hdma_spi_tx_, bufsize_);
        } else {
            HAL_SPI_Init(hspi_);
        }
    }
    uint32_t SPI::TransimiReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
        if (dma_) {
            // disable DMA
            __HAL_DMA_DISABLE(hdma_spi_rx_);
            __HAL_DMA_DISABLE(hdma_spi_tx_);
            while (hdma_spi_rx_->Instance->CR & DMA_SxCR_EN) {
                __HAL_DMA_DISABLE(hdma_spi_rx_);
            }
            while (hdma_spi_tx_->Instance->CR & DMA_SxCR_EN) {
                __HAL_DMA_DISABLE(hdma_spi_tx_);
            }
            // clear flag
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_->hdmarx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi_->hdmarx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi_->hdmarx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi_->hdmarx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi_->hdmarx));

            __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_->hdmatx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi_->hdmatx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi_->hdmatx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi_->hdmatx));
            __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi_->hdmatx));
            // set memory address
            hdma_spi_rx_->Instance->M0AR = (uint32_t)rx_data;
            hdma_spi_tx_->Instance->M0AR = (uint32_t)tx_data;
            // set data length
            __HAL_DMA_SET_COUNTER(hdma_spi_rx_, length);
            __HAL_DMA_SET_COUNTER(hdma_spi_tx_, length);
            // enable DMA
            __HAL_DMA_ENABLE(hdma_spi_rx_);
            __HAL_DMA_ENABLE(hdma_spi_tx_);
        } else {
            HAL_SPI_TransmitReceive_IT(hspi_, tx_data, rx_data, length);
        }
        return length;
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
        instance->callback_();
    }
    bool SPI::IsDMA() {
        return dma_;
    }
}  // namespace bsp

void RM_DMA_SPI_IRQHandler(SPI_HandleTypeDef* hspi) {
    bsp::SPI::CallbackWrapper(hspi);
}