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
#include <map>

#include "main.h"
#include "spi.h"
#include "bsp_gpio.h"

namespace bsp {

    class SPI;

   /* spi callback function pointer */
   typedef void (*spi_rx_callback_t)(SPI* spi);

   enum spi_mode_e {
       SPI_MODE_BLOCKED = 0,
       SPI_MODE_INTURRUPT = 1,
       SPI_MODE_DMA = 2,
   };

   typedef struct {
       SPI_HandleTypeDef* hspi;
       spi_mode_e mode;
   } spi_init_t;

   class SPI {
     public:
       /**
        * @brief constructor for spi instance
        *
        * @param hspi pointer to a HAL spi handle
        */
       explicit SPI(spi_init_t init);

       /**
        * @brief destructor (potentially deallocate buffer memories associated with
        * tx / rx)
        */
       virtual ~SPI();

       /**
        * @brief transmit and receive data
        * @param tx_data the data to be transmitted
        * @param rx_data the data to be received
        * @param length the size of the data
        * @param timeout the timeout for the transmission, only valid for blocked mode
        */
       void TransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length);

       /**
        * @brief transmit data
        * @param tx_data the data to be transmitted
        * @param length the size of the data
        * @param timeout the timeout for the transmission
        */
        void Transmit(uint8_t* tx_data, uint32_t length);

        /**
         * @brief receive data
         * @param rx_data the data to be received
         * @param length the size of the data
         */
        void Receive(uint8_t* rx_data, uint32_t length);

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



     protected:
       SPI_HandleTypeDef* hspi_;
       spi_mode_e mode_;
       spi_rx_callback_t callback_ = [](SPI* spi) { UNUSED(spi);};
       uint8_t rx_size_;
       uint8_t* rx_buffer_;


     private:
       static std::map<SPI_HandleTypeDef*, SPI*> ptr_map;
       static SPI* FindInstance(SPI_HandleTypeDef* hspi);
   };

   typedef void (*spi_master_rx_callback_t)();

   typedef struct {
       SPI* spi;
       GPIO* cs;
   } spi_master_init_t;

   enum spi_master_status_e {
       SPI_MASTER_STATUS_OK = 0,
       SPI_MASTER_STATUS_BUSY = 1,
       SPI_MASTER_STATUS_ERROR = 2
   };

   class SPIMaster{
     public:
        explicit SPIMaster(spi_master_init_t init);
        virtual ~SPIMaster();
        spi_master_status_e Transmit(uint8_t* tx_data, uint32_t length);
        spi_master_status_e Receive(uint8_t* rx_data, uint32_t length);
        spi_master_status_e TransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length);
        void RegisterCallback(spi_master_rx_callback_t callback);
        static void CallbackWrapper(SPI* spi);
     private:
        SPI* spi_;
        GPIO* cs_;
        spi_master_rx_callback_t callback_ = []() {};
   };
}  // namespace bsp
