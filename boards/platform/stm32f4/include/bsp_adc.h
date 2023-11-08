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

namespace bsp {

    /**
     * @brief ADC模拟量输入管理类
     * @details 用于读取ADC模拟量输入
     * @note    仅支持单通道单次采样
     * @note    由于名称ADC被占用，因此使用bADC
     */
    /**
     * @brief ADC analog input management class
     * @details Used to read ADC analog input
     * @note    Only single channel single sampling is supported
     * @note    Since the name ADC is occupied, bADC is used
     */
    class bADC {
      public:
        /**
         * @brief 构造函数
         * @param hadc hadc句柄
         * @param channel 通道
         * @param rank 通道序号
         * @param sampling_time 采样时间
         */
        /**
         * @brief constructor
         * @param hadc hadc handle
         * @param channel ADC Channel
         * @param rank channel rank
         * @param sampling_time sampling time
         */
        bADC(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank, uint32_t sampling_time);
        /**
         * @brief 准备采样数据
         */
        /**
         * @brief prepare sampling data
         */
        void Start();
        /**
         * @brief 停止采样
         */
        /**
         * @brief stop sampling
         */
        void Stop();
        /**
         * @brief 读取采样数据
         * @return 采样数据
         */
        /**
         * @brief read sampling data
         * @return sampling data
         */
        uint32_t Read();

      private:
        ADC_HandleTypeDef* hadc_;
        uint32_t channel_;
        uint32_t rank_;
        uint32_t sampling_time_;
    };
}  // namespace bsp