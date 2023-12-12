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

#include "usart.h"

namespace bsp {

    /**
     * @brief 串口管理类
     * @details 用于串口的收发
     */
    /**
     * @brief UART manager
     * @details used for UART send and receive
     */
    class UART {
      public:
        /**
         * @brief 构造函数，用于通用UART
         *
         * @param huart HAL uart 句柄
         */
        /**
         * @brief constructor for uart instance
         *
         * @param huart pointer to a HAL uart handle
         */
        explicit UART(UART_HandleTypeDef* huart);

        /**
         * @brief 析构函数（释放 tx / rx 相关的缓存内存）
         */
        /**
         * @brief destructor (potentially deallocate buffer memories associated with
         * tx / rx)
         */
        virtual ~UART();

        /**
         * @brief 设置接收回调函数与缓存
         *
         * @param rx_buffer_size 接收缓存大小（未读取的数据会被缓存到此缓存中）
         */
        /**
         * @brief set up uart receiver in the background optionally registering a
         * callback
         *
         * @param rx_buffer_size  receive buffer size (all data that has not been
         * read out is queued into this buffer)
         */
        void SetupRx(uint32_t rx_buffer_size, bool dma = true);

        /**
         * @brief 设置非阻塞发送功能
         *
         * @param tx_buffer_size  发送缓存大小（发送调用会被缓存到此缓存中）
         * @param dma 是否使用DMA
         */
        /**
         * @brief set up non blocking transmission functionality
         *
         * @param tx_buffer_size  transmission buffer size (burst transmission calls
         * will be queued into this buffer)
         * @param dma whether to use DMA
         */
        void SetupTx(uint32_t tx_buffer_size, bool dma = true);

        /**
         * @brief 读取接收到的数据
         * @tparam FromISR  设置为 true 以在中断处理程序中调用
         * @param data  指向数组地址的指针，该数组地址被设置为接收缓冲区地址
         * @return 读取的字节数，失败返回-1
         *
         * @note 为了最佳性能，不会复制内存，因此对此方法的第二次调用将使前一次调用产生的缓冲区无效
         */
        /**
         * @brief read out the pending received data
         *
         * @tparam FromISR  set to true to call inside an interrupt handler
         * @param data  pointer to an array address that gets set to the receive
         * buffer address
         *
         * @return number of bytes read, -1 if failure
         *
         * @note memory is not copied for optimal performance, so second call to this
         *       method will invalidate the buffer produced by the previous call
         */
        template <bool FromISR = false>
        int32_t Read(uint8_t** data);

        /**
         * @brief 写数据到串口
         *
         * @tparam FromISR  设置为 true 以在中断处理程序中调用
         * @param data  指向要传输的数据缓冲区的指针
         * @param length  要传输的数据的长度
         * @return 写入的字节数
         *
         * @note 多次调用此函数可能会导致 tx 缓冲区填满，因此请记住检查实际传输的字节数的返回值
         *      以避免数据丢失
         *
         */
        /**
         * @brief write data to uart without blocking
         *
         * @tparam FromISR  set to true to call inside an interrupt handler
         * @param data    pointer to the data buffer to be transmitted
         * @param length  length of the data to be transmitted
         *
         * @return number of bytes written
         *
         * @note multiple burst calls to this function can potentially cause tx
         * buffer to fill up, so remember to check return value for the actual number
         *       of bytes successfully transmitted
         */
        template <bool FromISR = false>
        int32_t Write(const uint8_t* data, uint32_t length);

        /**
         * @brief 修改串口波特率
         * @param baudrate 波特率
         * @note 仅在串口初始化之前调用有效
         */
        /**
         * @brief change baudrate
         * @param baudrate baudrate
         * @note only valid before uart initialization
         */
         void SetBaudrate(uint32_t baudrate);

      protected:
        /**
         * @brief 串口发送完成回调函数
         */
        /**
         * @brief Transmission complete call back.
         */
        void TxCompleteCallback();
        /**
         * @brief 串口接收完成回调函数
         */
        /**
         * @brief Reception complete call back.
         */
        virtual void RxCompleteCallback();

        UART_HandleTypeDef* huart_;
        /* rx */
        uint32_t rx_size_;
        uint8_t* rx_data_[2];
        uint8_t rx_index_;
        /* tx */
        uint32_t tx_size_;
        uint32_t tx_pending_;
        uint8_t* tx_write_;
        uint8_t* tx_read_;
        bool tx_dma_ = true;
        bool rx_dma_ = true;

      private:
        friend void RxCompleteCallbackWrapper(UART_HandleTypeDef* huart);
        friend void TxCompleteCallbackWrapper(UART_HandleTypeDef* huart);

        static std::map<UART_HandleTypeDef*, UART*> ptr_map;
        static UART* FindInstance(UART_HandleTypeDef* huart);
    };

} /* namespace bsp */
