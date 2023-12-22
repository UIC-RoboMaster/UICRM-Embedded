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

#include "bsp_gpio.h"
#include "main.h"
#include "spi.h"

#define SPI_MAX_DEVICE 6

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

    /**
     * @brief SPI管理类
     * @details 用于SPI的收发
     */
    /**
     * @brief SPI manager
     * @details used for SPI send and receive
     */
    class SPI {
      public:
        /**
         * @brief 构造函数
         *
         * @param init SPI的初始化配置
         */
        /**
         * @brief constructor for spi instance
         *
         * @param init Init config of SPI
         */
        explicit SPI(spi_init_t init);

        /**
         * @brief destructor (potentially deallocate buffer memories associated with
         * tx / rx)
         */
        virtual ~SPI();

        /**
         * @brief 同时收发SPI数据
         * @param tx_data 需要被发送的数据
         * @param rx_data 需要被接收的数据
         * @param length 数据长度
         */
        /**
         * @brief transmit and receive data
         * @param tx_data the data to be transmitted
         * @param rx_data the data to be received
         * @param length the size of the data
         */
        void TransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length);

        /**
         * @brief 发送SPI数据
         * @param tx_data 需要被发送的数据
         * @param length 数据长度
         */
        /**
         * @brief transmit data
         * @param tx_data the data to be transmitted
         * @param length the size of the data
         */
        void Transmit(uint8_t* tx_data, uint32_t length);

        /**
         * @brief 接收SPI数据
         * @param rx_data 需要被接收的数据
         * @param length 数据长度
         */
        /**
         * @brief receive data
         * @param rx_data the data to be received
         * @param length the size of the data
         */
        void Receive(uint8_t* rx_data, uint32_t length);

        /**
         * @brief 检测SPI是否忙碌
         * @return 如果忙碌则返回true
         */
        /**
         * @brief check if the spi is busy
         * @return true if the spi is busy
         */
        bool IsBusy();

        /**
         * @brief 中断SPI传输
         */
        /**
         * @brief Abort SPI transmission
         */
        void Abort();

        /**
         * @brief 设置SPI的传输模式
         * @param mode 被设置的模式
         */
        /**
         * @brief Set the SPI Mode
         * @param mode the mode to be set
         */
        void SetMode(spi_mode_e mode);

        /**
         * @brief 注册一个回调函数，当中断/DMA的数据收发完成之后，这个函数会被调用。
         * @param callback 被调用的函数
         */
        /**
         * @brief register a callback function to be called when the spi receives data
         * @param callback the function will be called
         */
        void RegisterCallback(spi_rx_callback_t callback);

        /**
         * @brief 检测是否使用DMA
         * @return 如果使用DMA则返回true
         */
        /**
         * @brief Check Using DMA or not
         * @return true if using DMA
         */
        bool IsDMA();

        /**
         * @brief 回调调用函数，被HAL库调用
         * @param hspi hspi句柄
         */
        /**
         * @brief the function used for HAL callback.
         * @param hspi hspi handle
         */
        static void CallbackWrapper(SPI_HandleTypeDef* hspi);

      protected:
        SPI_HandleTypeDef* hspi_;
        spi_mode_e mode_;
        spi_rx_callback_t callback_ = [](SPI* spi) { UNUSED(spi); };
        uint8_t rx_size_;
        uint8_t* rx_buffer_;

      private:
        static std::unordered_map<SPI_HandleTypeDef*, SPI*> ptr_map;
        static SPI* FindInstance(SPI_HandleTypeDef* hspi);
    };

    typedef void (*spi_device_rx_callback_t)(void* args);

    typedef struct {
        SPI* spi;
        GPIO* cs;
    } spi_device_init_t;

    /**
     * @brief SPI设备
     * @details 拥有一个CS引脚的SPI设备，需要被SPIMaster类调用
     */
    /**
     * @brief SPI Device
     * @details A SPI device with a CS port, need to be called at the class SPIMaster
     */
    class SPIDevice {
      public:
        /**
         * @brief 构造函数
         *
         * @param init SPI设备的初始化配置
         */
        /**
         * @brief constructor for spi instance
         *
         * @param init Init config of SPI Device
         */
        SPIDevice(spi_device_init_t init);
        /**
         * @brief 注册一个回调函数，当中断/DMA的数据收发完成之后，这个函数会被调用。
         * @param callback 被调用的函数
         */
        /**
         * @brief register a callback function to be called when the spi receives data
         * @param callback the function will be called
         */
        void RegisterCallback(spi_device_rx_callback_t callback,void* args=NULL);
        void PrepareTransmit();
        void FinishTransmit();
        bool IsTransmitting();
        void CallbackWrapper();
        SPI* GetSPI() {
            return spi_;
        }

      private:
        SPI* spi_;
        GPIO* cs_;
        spi_device_rx_callback_t callback_ = [](void* args) { UNUSED(args);};
        void* args_ = NULL;
    };

    typedef struct {
        SPI* spi;
    } spi_master_init_t;

    enum spi_master_status_e {
        SPI_MASTER_STATUS_OK = 0,
        SPI_MASTER_STATUS_BUSY = 1,
        SPI_MASTER_STATUS_ERROR = 2
    };

    /**
     * @brief 管理SPI和连接对应设备的类
     * @details 使SPI设备能正常使用与回调
     */
    /**
     * @brief the Class for manage the SPI and connected devices
     * @details Let SPI devices to be used normally and callback
     */
    class SPIMaster {
      public:
        /**
         * @brief 构造函数
         *
         * @param init SPIMaster的初始化配置
         */
        /**
         * @brief constructor for spi instance
         *
         * @param init Init config of SPIMaster
         */
        explicit SPIMaster(spi_master_init_t init);
        virtual ~SPIMaster();
        /**
         * @brief 新建一个SPI设备
         * @param cs SPI设备所使用的CS端口
         * @return SPI设备类
         */
        /**
         * @brief New SPI Device
         * @param cs the cs GPIO Port is used
         * @return SPI Device Class
         */
        SPIDevice* NewDevice(GPIO* cs);
        /**
         * @brief 添加SPI设备到SPIMaster类
         * @param device 需要被添加的SPI设备
         */
        /**
         * @brief Add SPI Device to SPIMaster Class
         * @param device the device need to be add
         */
        void AddDevice(SPIDevice* device);
        /**
         * @brief 检测SPI是否忙碌
         * @return 如果忙碌则返回true
         */
        /**
         * @brief check if the spi is busy
         * @return true if the spi is busy
         */
        bool IsBusy();
        /**
         * @brief 发送SPI数据
         * @param device SPI设备
         * @param tx_data 需要被发送的数据
         * @param length 数据长度
         * @return 发送状态
         */
        /**
         * @brief transmit data
         * @param device SPI Device
         * @param tx_data the data to be transmitted
         * @param length the size of the data
         * @return Send status
         */
        spi_master_status_e Transmit(SPIDevice* device, uint8_t* tx_data, uint32_t length);
        /**
         * @brief 接收SPI数据
         * @param device SPI设备
         * @param rx_data 需要被接收的数据
         * @param length 数据长度
         * @return 发送状态
         */
        /**
         * @brief receive data
         * @param device SPI Device
         * @param rx_data the data to be received
         * @param length the size of the data
         * @return Send status
         */
        spi_master_status_e Receive(SPIDevice* device, uint8_t* rx_data, uint32_t length);
        /**
         * @brief 同时收发SPI数据
         * @param device SPI设备
         * @param tx_data 需要被发送的数据
         * @param rx_data 需要被接收的数据
         * @param length 数据长度
         * @return 发送状态
         */
        /**
         * @brief transmit and receive data
         * @param device SPI Device
         * @param tx_data the data to be transmitted
         * @param rx_data the data to be received
         * @param length the size of the data
         * @return Send status
         */
        spi_master_status_e TransmitReceive(SPIDevice* device, uint8_t* tx_data, uint8_t* rx_data,
                                            uint32_t length);
        /**
         * @brief 设置SPI的传输模式
         * @param mode 被设置的模式
         */
        /**
         * @brief Set the SPI Mode
         * @param mode the mode to be set
         */
        void SetMode(spi_mode_e mode);
        /**
         * @brief 设置是否自动拉低CS引脚
         * @param auto_cs true则自动拉低CS引脚
         */
        /**
         * @brief Set automatic pull-down the CS Port
         * @param auto_cs true to auto pull-down the port
         */
        void SetAutoCS(bool auto_cs);
        /**
         * @brief 回调调用函数，被SPI类调用
         * @param spi SPI类
         */
        /**
         * @brief the function used for SPI callback.
         * @param spi SPI class
         */
        static void CallbackWrapper(SPI* spi);
        void CallbackWrapper();

      private:
        SPI* spi_;
        SPIDevice* device_[SPI_MAX_DEVICE] = {NULL};
        uint8_t device_count_ = 0;
        bool auto_cs_ = true;
        static std::unordered_map<SPI*, SPIMaster*> ptr_map;
        static SPIMaster* FindInstance(SPI* spi);
    };
}  // namespace bsp
