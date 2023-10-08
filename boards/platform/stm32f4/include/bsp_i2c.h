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

#include "i2c.h"
#include "main.h"
#define MAX_I2C_DEVICES 24
#define MAX_I2C_DATA_SIZE 8

namespace bsp {

    /* i2c callback function pointer */
    typedef void (*i2c_rx_callback_t)(const uint8_t data[], void* args);

    class I2C {
      public:
        /**
         * @brief constructor for bsp CAN instance
         *
         * @param hi2c     HAL can handle
         */
        I2C(I2C_HandleTypeDef* hi2c,bool is_master = true, bool is_dma = false);
        /**
         * @brief check if it is associated with a given CAN handle
         *
         * @param hi2c  HAL can handle to be checked
         *
         * @return true if associated, otherwise false
         */
        bool Uses(I2C_HandleTypeDef* hi2c) {
            return hi2c_ == hi2c;
        }


        /**
         * @brief check the device is ready.
         *
         * @param id  device address
         *
         * @return true if ready, otherwise false
         */
        bool isReady(uint16_t id, uint32_t timeout=50);

        /**
         * @brief register callback function for a specific ID on this I2C line
         *
         * @param std_id    rx id
         * @param callback  callback function
         * @param args      argument passed into the callback function
         *
         * @return return 0 if success, -1 if invalid std_id
         */
        int RegisterRxCallback(uint32_t std_id, i2c_rx_callback_t callback, void* args = NULL);

        /**
         * @brief transmit I2C messages
         *
         * @param id      device address
         * @param data[]  data bytes
         * @param length  length of data
         *
         * @return  number of bytes transmitted, -1 if failed
         */
        int Transmit(uint16_t id, const uint8_t data[], uint16_t length);

        /**
         * @brief receive I2C messages
         *
         * @param id      device address
         * @param data[]  data bytes
         * @param length  length of data
         *
         * @return  number of bytes transmitted, -1 if failed
         */
        int Receive(uint16_t id, uint8_t* data, uint16_t length);


        /**
         * @brief read I2C memory register
         *
         * @param id     device address
         * @param reg    register address
         * @param data*  pointer to data buffer
         * @param length length of data
         *
         */
        int MemoryRead(uint16_t id, uint16_t reg, uint8_t* data, uint16_t length);

        /**
         * @brief read I2C memory register
         *
         * @param id     device address
         * @param reg    register address
         * @param data*  pointer to data buffer
         * @param length length of data
         *
         */
        int MemoryRead(uint16_t id, uint16_t reg, uint16_t* data, uint16_t length);

        /**
         * @brief write I2C memory register
         *
         * @param id     device address
         * @param reg    register address
         * @param data*  pointer to data buffer
         * @param length length of data
         */
        int MemoryWrite(uint16_t id, uint16_t reg, uint8_t* data, uint16_t length);

        /**
         * @brief write I2C memory register
         *
         * @param id     device address
         * @param reg    register address
         * @param data*  pointer to data buffer
         * @param length length of data
         */
        int MemoryWrite(uint16_t id, uint16_t reg, uint16_t* data, uint16_t length);

        /**
         * @brief callback wrapper called from IRQ context
         *
         * @note should not be called explicitly form the application side
         */
        void RxCallback();

        static I2C* FindInstance(I2C_HandleTypeDef* hi2c);

      private:

        I2C_HandleTypeDef* hi2c_;
        bool is_master_;
        bool is_dma_;


        i2c_rx_callback_t rx_callbacks_[MAX_I2C_DEVICES] = {0};
        void* rx_args_[MAX_I2C_DEVICES] = {NULL};

        std::map<uint16_t, uint8_t> id_to_index_;
        uint8_t callback_count_ = 0;

        static std::map<I2C_HandleTypeDef*, I2C*> ptr_map;
        static bool HandleExists(I2C_HandleTypeDef* hi2c);
        static void I2CRxCallback(I2C_HandleTypeDef* hi2c);

        uint8_t rx_data_[MAX_I2C_DATA_SIZE] = {0};
    };

} /* namespace bsp */