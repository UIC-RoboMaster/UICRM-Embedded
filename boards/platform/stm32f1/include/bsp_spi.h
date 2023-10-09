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
#pragma once
#include "main.h"
#include "spi.h"
#include <map>

namespace bsp{

    /* spi callback function pointer */
    typedef void (*spi_rx_callback_t)();

    class SPI {
      public:
        /**
          * @brief constructor for spi instance
          *
          * @param hspi pointer to a HAL spi handle
         */
        explicit SPI(SPI_HandleTypeDef* hspi);

        /**
          * @brief destructor (potentially deallocate buffer memories associated with
          * tx / rx)
         */
        virtual ~SPI();

        /**
         * @brief setup the buffer for spi transmission
         * @param buffer_size the size of the buffer
         * @param dma whether to use dma
         */
         void Setup(uint32_t buffer_size, bool dma = true);

         /**
          * @brief initialize the spi transmission
          */
         void Init();

         /**
          * @brief transmit and receive data
          * @param tx_data the data to be transmitted
          * @param rx_data the data to be received
          * @param length the size of the data
          * @param timeout the timeout for the transmission
          * @return the length of the data received
          */
         uint32_t TransimiReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length);


         /**
          * @brief check if the spi is busy
          * @return true if the spi is busy
          */
         bool IsBusy();

         /**
          * @brief register a callback function to be called when the spi receives data
          * @param callback
          */
         void RegisterCallback(spi_rx_callback_t callback);

         bool IsDMA();

         static void CallbackWrapper(SPI_HandleTypeDef* hspi);

         SPI_HandleTypeDef* hspi_;

       protected:
         DMA_HandleTypeDef* hdma_spi_rx_;
         DMA_HandleTypeDef* hdma_spi_tx_;
         uint32_t bufsize_;
         uint8_t* txbuf_;
         uint8_t* rxbuf_;
         bool dma_ = true;
         spi_rx_callback_t callback_ = []() {};;

       private:
         static std::map<SPI_HandleTypeDef*, SPI*> ptr_map;
         static SPI* FindInstance(SPI_HandleTypeDef* hspi);
    };
}

