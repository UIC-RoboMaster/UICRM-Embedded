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

#include <unordered_map>

#include "bsp_error_handler.h"
#include "can.h"

#define MAX_CAN_DATA_SIZE 8
#define MAX_CAN_DEVICES 24

namespace bsp {

    /**
     * @brief CAN接收回调函数
     */
    /**
     * @brief callback function for can rx
     */
    typedef void (*can_rx_callback_t)(const uint8_t data[], void* args);

    /**
     * @brief CAN接收回调函数
     */
    /**
     * @brief callback function for can rx
     */
    typedef void (*can_rx_ext_callback_t)(const uint8_t data[], const uint32_t ext_id, void* args);

    /**
     * @brief CAN管理类
     * @details 用于CAN的收发
     */
    /**
     * @brief CAN manager
     * @details used for CAN send and receive
     */
    class CAN {
      public:
        /**
         * @brief bsp CAN实例构造函数
         *
         * @param hcan     HAL can 句柄
         * @param start_id rx的最小stdid，当前已经弃用
         * @param is_master
         * 是否为主控端，此项在hcan为hcan1的时候需要设置为true，其他时候设置为false。
         */
        /**
         * @brief constructor for bsp CAN instance
         *
         * @param hcan     HAL can handle
         * @param is_master whether this is the master node, set to true if hcan is hcan1, otherwise
         * false
         */
        CAN(CAN_HandleTypeDef* hcan, bool is_master = true, uint8_t ext_id_suffix = 8);
        /**
         * @brief 检查是否与给定的CAN句柄相关联
         *
         * @return 如果关联则返回true，否则返回false
         */
        /**
         * @brief check if it is associated with a given CAN handle
         *
         * @param hcan  HAL can handle to be checked
         *
         * @return true if associated, otherwise false
         */
        bool Uses(CAN_HandleTypeDef* hcan) {
            return hcan_ == hcan;
        }

        /**
         * @brief 注册CAN接收回调函数
         *
         * @param std_id    需要注册的rx id
         * @param callback  回调函数
         * @param args      传入回调函数的参数
         *
         * @return 如果注册成功返回0，否则返回-1
         */
        /**
         * @brief register callback function for a specific ID on this CAN line
         *
         * @param std_id    rx id
         * @param callback  callback function
         * @param args      argument passed into the callback function
         *
         * @return return 0 if success, -1 if invalid std_id
         */
        int RegisterRxCallback(uint32_t std_id, can_rx_callback_t callback, void* args = NULL);

        /**
         * @brief 注册CAN接收回调函数
         *
         * @param ext_id_suffix    需要注册的rx id
         * @param callback  回调函数
         * @param args      传入回调函数的参数
         *
         * @return 如果注册成功返回0，否则返回-1
         */
        /**
         * @brief register callback function for a specific ID on this CAN line
         *
         * @param ext_id_suffix    rx id
         * @param callback  callback function
         * @param args      argument passed into the callback function
         *
         * @return return 0 if success, -1 if invalid std_id
         */
        int RegisterRxExtendCallback(uint32_t ext_id_suffix, can_rx_ext_callback_t callback,
                                     void* args = NULL);

        /**
         * @brief 发送CAN数据
         *
         * @param id      tx id
         * @param data[]  数据
         * @param length  数据长度，必须在(0, 8]之间
         *
         * @return  返回发送的字节数，如果发送失败返回-1
         */
        /**
         * @brief transmit can messages
         *
         * @param id      tx id
         * @param data[]  data bytes
         * @param length  length of data, must be in (0, 8]
         *
         * @return  number of bytes transmitted, -1 if failed
         */
        int Transmit(uint16_t id, const uint8_t data[], uint32_t length);

        /**
         * @brief 发送CAN数据，使用扩展Can ID
         *
         * @param id      tx id
         * @param data[]  数据
         * @param length  数据长度，必须在(0, 8]之间
         *
         * @return  返回发送的字节数，如果发送失败返回-1
         */
        /**
         * @brief transmit can messages, using extended can id
         *
         * @param id      tx id
         * @param data[]  data bytes
         * @param length  length of data, must be in (0, 8]
         *
         * @return  number of bytes transmitted, -1 if failed
         */
        int TransmitExtend(uint32_t id, const uint8_t data[], uint32_t length);

        /**
         * @brief CAN的接收回调
         *
         * @note 该函数不应该被用户调用
         */
        /**
         * @brief callback wrapper called from IRQ context
         *
         * @note should not be called explicitly form the application side
         */
        void RxCallback();

        /**
         * @brief CAN的扩展ID接收回调
         *
         * @note 该函数不应该被用户调用
         */
        /**
         * @brief callback wrapper called from IRQ context
         *
         * @note should not be called explicitly form the application side
         */
        void RxExtendCallback(CAN_RxHeaderTypeDef header, uint8_t* data);

      private:
        void ConfigureFilter(bool is_master);

        CAN_HandleTypeDef* hcan_;

        can_rx_callback_t rx_callbacks_[MAX_CAN_DEVICES] = {0};
        void* rx_args_[MAX_CAN_DEVICES] = {NULL};

        std::unordered_map<uint16_t, uint8_t> id_to_index_;
        uint8_t callback_count_ = 0;

        can_rx_ext_callback_t rx_ext_callbacks_[MAX_CAN_DEVICES] = {0};
        void* rx_ext_args_[MAX_CAN_DEVICES] = {NULL};

        std::unordered_map<uint32_t, uint8_t> ext_to_index_;
        uint8_t ext_callback_count_ = 0;

        uint8_t ext_id_suffix_;

        static std::unordered_map<CAN_HandleTypeDef*, CAN*> ptr_map;
        static CAN* FindInstance(CAN_HandleTypeDef* hcan);
        static bool HandleExists(CAN_HandleTypeDef* hcan);
        static void RxFIFO0MessagePendingCallback(CAN_HandleTypeDef* hcan);
    };

} /* namespace bsp */