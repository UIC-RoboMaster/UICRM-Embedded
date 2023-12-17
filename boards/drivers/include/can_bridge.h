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
#include "bsp_can.h"
#include <unordered_map>

#define MAX_CAN_BRIDGE_REG 32

namespace communication {
    enum can_bridge_type_e{
        CAN_BRIDGE_PING = 0x00,
        CAN_BRIDGE_TYPE_INT64 = 0x01,
        CAN_BRIDGE_TYPE_UINT64 = 0x02,
        CAN_BRIDGE_TYPE_DOUBLE = 0x03,
        CAN_BRIDGE_TYPE_INT32 = 0x04,
        CAN_BRIDGE_TYPE_UINT32 = 0x05,
        CAN_BRIDGE_TYPE_TWO_INT32 = 0x06,
        CAN_BRIDGE_TYPE_TWO_UINT32 = 0x07,
        CAN_BRIDGE_TYPE_FLOAT = 0x08,
        CAN_BRIDGE_TYPE_TWO_FLOAT = 0x09,
        CAN_BRIDGE_TYPE_INT16 = 0x0A,
        CAN_BRIDGE_TYPE_UINT16 = 0x0B,
        CAN_BRIDGE_TYPE_TWO_INT16 = 0x0C,
        CAN_BRIDGE_TYPE_TWO_UINT16 = 0x0D,
        CAN_BRIDGE_TYPE_FOUR_INT16 = 0x0E,
        CAN_BRIDGE_TYPE_FOUR_UINT16 = 0x0F,
        CAN_BRIDGE_TYPE_EIGHT_INT8 = 0x10,
        CAN_BRIDGE_TYPE_EIGHT_UINT8 = 0x11,
        CAN_BRIDGE_EXTEND_INT32 = 0x19,
        CAN_BRIDGE_EXTEND_UINT32 = 0x1A,
        CAN_BRIDGE_EXTEND_FLOAT = 0x1B,
        CAN_BRIDGE_EXTEND_TWO_INT16 = 0x1C,
        CAN_BRIDGE_EXTEND_TWO_UINT16 = 0x1D,
        CAN_BRIDGE_ERROR = 0x1E,
        CAN_BRIDGE_PONG = 0x1F,
    };


    typedef union{
     struct {
         uint16_t rx_id : 8;
         uint16_t tx_id : 8;
         uint16_t reg : 8;
         uint16_t type:5;
         uint8_t res : 3;
     }__packed data;
     uint32_t id;
    } __packed can_bridge_ext_id_t;

    typedef union {
        uint8_t data[8];
        struct {
            int64_t data;
        }__packed data_int64;
        struct {
            uint64_t data;
        }__packed data_uint64;
        struct {
            double data;
        }__packed data_double;
        struct {
            int32_t data;
            uint32_t res;
        }__packed data_int32;
        struct {
            uint32_t data;
            uint32_t res;
        }__packed data_uint32;
        struct {
            int32_t data[2];
        }__packed data_two_int32;
        struct {
            uint32_t data[2];
        }__packed data_two_uint32;
        struct {
            float data;
            uint32_t res;
        }__packed data_float;
        struct {
            float data[2];
        }__packed data_two_float;
        struct {
            int16_t data;
            uint64_t res:48;
        }__packed data_int16;
        struct {
            uint16_t data;
            uint64_t res:48;
        }__packed data_uint16;
        struct {
            int16_t data[2];
            uint64_t res:32;
        }__packed data_two_int16;
        struct {
            uint16_t data[2];
            uint64_t res:32;
        }__packed data_two_uint16;
        struct {
            int16_t data[4];
        }__packed data_four_int16;
        struct {
            uint16_t data[4];
        }__packed data_four_uint16;
        struct {
            int8_t data[8];
        }__packed data_eight_int8;
        struct {
            uint8_t data[8];
        }__packed data_eight_uint8;
        struct {
            uint32_t extendreg;
            int32_t data;
        }__packed data_extend_int32;
        struct {
            uint32_t extendreg;
            uint32_t data;
        }__packed data_extend_uint32;
        struct {
            uint32_t extendreg;
            float data;
        }__packed data_extend_float;
        struct {
            uint32_t extendreg;
            int16_t data[2];
        }__packed data_extend_two_int16;
        struct {
            uint32_t extendreg;
            uint16_t data[2];
        }__packed data_extend_two_uint16;
        struct {
            uint64_t error;
        }__packed data_error;
    }__packed can_bridge_data_t;

    typedef void (*can_bridge_rx_callback_t)(can_bridge_ext_id_t ext_id, can_bridge_data_t data,void* args);

    class CanBridge {
      public:
        CanBridge(bsp::CAN* can,uint8_t id);
        ~CanBridge()=default;
        void Send(can_bridge_ext_id_t ext_id, can_bridge_data_t data);

        void RegisterRxCallback(uint8_t reg,can_bridge_rx_callback_t callback,void* args = NULL);

        void CallbackWrapper(const uint8_t data[],const uint32_t ext_id);

      private:
        bsp::CAN* can_;
        uint8_t id_;

        can_bridge_rx_callback_t reg_callbacks_[MAX_CAN_BRIDGE_REG] = {0};
        void* reg_args_[MAX_CAN_BRIDGE_REG] = {NULL};
        std::unordered_map<uint8_t, uint8_t> reg_to_index_;
        uint8_t reg_callback_count_ = 0;
    };
}
