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

#include "bsp_error_handler.h"
#include "main.h"

#define NUM_GPITS 16

namespace bsp {

    /**
     * @brief GPIO管理类
     * @details 用于GPIO的输入输出
     */
    /**
     * @brief GPIO manager
     * @details used for GPIO input and output
     */
    class GPIO {
      public:
        /**
         * @brief 构造函数，用于通用GPIO（非中断）
         *
         * @param group GPIO组
         * @param pin   GPIO引脚号
         */
        /**
         * @brief constructor for generic GPIO (non interrupt)
         *
         * @param group GPIO group
         * @param pin   GPIO pin number
         */
        GPIO(GPIO_TypeDef* group, uint16_t pin);

        /**
         * @brief 设置高电平输出
         */
        /**
         * @brief Set high output
         */
        void High();

        /**
         * @brief 设置低电平输出
         */
        /**
         * @brief Set low output
         */
        void Low();

        /**
         * @brief 翻转GPIO输出
         */
        /**
         * @brief Toggle GPIO output
         */
        void Toggle();

        /**
         * @brief 读取GPIO输入
         *
         * @return 1为高电平，0为低电平
         */
        /**
         * @brief Read GPIO input
         *
         * @return 1 for high and 0 for low
         */
        uint8_t Read();

      private:
        GPIO_TypeDef* group_;
        uint16_t pin_;
        uint8_t state_;
    };

    typedef void (*gpit_callback_t)(void* args);

    /**
     * @brief 通用中断引脚管理类
     * @details 用于通用中断引脚的管理
     */
    /**
     * @brief General purpose interrupt pin manager
     * @details used for general purpose interrupt pin management
     */
    class GPIT {
      public:
        /**
         * @brief 构造函数，用于通用中断引脚
         *
         * @param pin 中断引脚号（EXTIx中的x）
         */
        /**
         * @brief Contructor for general purpose interrupt pins
         *
         * @param pin interrupt pin number (x in EXTIx)
         */
        GPIT(uint16_t pin);

        /**
         * @brief 注册中断回调函数
         */
        /**
         * @brief register interrupt callback function
         */
        void RegisterCallback(gpit_callback_t callback, void* args = nullptr) {
            callback_ = callback;
            args_ = args;
        };

        /**
         * @brief 虚拟中断回调函数，需要在子类中实现。
         */
        /**
         * @brief Callback back when interrupt happens
         */
        virtual void IntCallback() {
            callback_(args_);
        };

        /**
         * @brief 全局中断回调函数
         *
         * @param pin 中断引脚号
         */
        /**
         * @brief wrapper for global interrupt handler
         *
         * @param pin interrupt pin number
         */
        static void IntCallback(uint16_t pin);

      private:
        static int GetGPIOIndex(uint16_t pin);

        static GPIT* gpits[NUM_GPITS];

        uint16_t pin_;

        gpit_callback_t callback_ = [](void* args) { UNUSED(args); };

        void* args_ = nullptr;
    };

} /* namespace bsp */
