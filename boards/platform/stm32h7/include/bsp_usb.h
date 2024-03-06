/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#ifndef NO_USB
#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

namespace bsp {

    /**
     * @brief USB虚拟串口类
     * @details 用于USB虚拟串口的数据收发
     */
    class VirtualUSB {
      public:
        /**
         * @brief 构造函数
         *
         */
        /**
         * @brief constructor for usb instance
         *
         * @param true if use callback for receiving, otherwise manual read is
         * required. The default value is false
         */
        explicit VirtualUSB();

        /**
         * @brief 析构函数
         * @details 可能会释放tx/rx相关的buffer内存
         */
        /**
         * @brief destructor (potentially deallocate buffer memories associated with
         * tx / rx)
         */
        virtual ~VirtualUSB();

        /**
         * @brief 初始化非阻塞USB串口传输
         *
         * @param tx_buffer_size 传输缓冲区大小（burst传输调用将被排队到此缓冲区）
         */
        /**
         * @brief set up usb non blocking transmission
         *
         * @param tx_buffer_size  transmission buffer size (burst transmission calls
         * will be queued into this buffer)
         */
        void SetupTx(uint32_t tx_buffer_size);

        /**
         * @brief 初始化USB接收器
         *
         * @param rx_buffer_size
         * 接收缓冲区大小（所有未读取的数据都将排队到此缓冲区，如果没有注册回调）
         */
        /**
         * @brief set up usb receiver
         *
         * @param rx_buffer_size  receive buffer size (all data that has not been
         * read out is queued into this buffer if there is no callbacks registered)
         */
        void SetupRx(uint32_t rx_buffer_size);

        /**
         * @brief 读取USB接收到的数据
         *
         * @param data  指向接收缓冲区地址的数组地址的引用
         *
         * @return 读取的字节数
         *
         * @note 为了最佳性能，不会复制内存，因此对此方法的第二次调用将使前一次调用产生的缓冲区无效
         */
        /**
         * @brief read out the pending received data
         *
         * @param data  reference to an array address that gets set to the receive
         * buffer address
         *
         * @return number of bytes read
         *
         * @note memory is not copied for optimal performance, so second call to this
         *       method will invalidate the buffer produced by the previous call
         */
        uint32_t Read(uint8_t** data);

        /**
         * @brief 非阻塞发送数据到USB
         *
         * @param data  指向要传输的数据缓冲区的指针
         * @param length  要传输的数据的长度
         *
         * @return 写入的字节数
         *
         * @note
         * 对此函数的多个burst调用可能会导致tx缓冲区填满，因此请记住检查返回值以获取实际传输的字节数
         */
        /**
         * @brief write data to usb without blocking
         *
         * @param data    pointer to the data buffer to be transmitted
         * @param length  length of the data to be transmitted
         *
         * @return number of bytes written
         *
         * @note multiple burst calls to this function can potentially cause tx
         * buffer to fill up, so remember to check return value for the actual number
         *       of bytes successfully transmitted
         */
        uint32_t Write(uint8_t* data, uint32_t length);

      protected:
        /**
         * @brief 传输完成回调函数
         */
        /**
         * @brief Transmission complete callback function
         */
        void TxCompleteCallback();

        /**
         * @brief 接收完成回调函数
         */
        /**
         * @brief Reception complete call back
         *
         * @param data    pointer to the data buffer received
         * @param length  length of the data received
         */
        virtual void RxCompleteCallback();

        /**
         * @brief 队列接收到的数据（如果未使用回调）
         *
         * @param data  指向接收到的数据缓冲区的指针
         * @param length  接收到的数据长度
         *
         * @return 实际写入缓冲区的字节数
         */
        /**
         * @brief Queue up received data if callback is not used
         *
         * @param data    pointer to the data buffer received
         * @param length  length of the data received
         *
         * @return number of bytes actually written to the buffer
         */
        uint32_t QueueUpRxData(const uint8_t* data, uint32_t length);

      public:
        /**
         * @brief USB传输完成回调函数
         *
         */
        /**
         * @brief Wrapper function of transmission complete callback
         *
         * @param data    pointer to the data buffer received
         * @param length  length of the data received
         */
        friend void TxCompleteCallbackWrapper();

        /**
         * @brief USB接收完成回调函数
         *
         * @param data    指向接收到的数据缓冲区的指针
         * @param length  接收到的数据长度
         */
        /**
         * @brief Wrapper function of reception complete callback
         *
         * @param data    pointer to the data buffer received
         * @param length  length of the data received
         */
        friend void RxCompleteCallbackWrapper(uint8_t* data, uint32_t length);

      protected:
        /* rx */
        uint32_t rx_size_;
        uint32_t rx_pending_;
        uint8_t* rx_write_;
        uint8_t* rx_read_;
        /* tx */
        uint32_t tx_size_;
        uint32_t tx_pending_;
        uint8_t* tx_write_;
        uint8_t* tx_read_;
    };

} /* namespace bsp */
#endif
