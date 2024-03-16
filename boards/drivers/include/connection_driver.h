/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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

#include <stdint.h>
namespace driver {

    class ConnectionDriver {
      public:
        ConnectionDriver() = default;
        explicit ConnectionDriver(uint32_t online_threshold) : online_threshold_(online_threshold) {}
        virtual ~ConnectionDriver() = default;

        /**
         * @brief 判断节点是否在线
         * @return true 表示当前连接节点在线，false 表示当前节点连接离线
         */
        bool IsOnline() const;

        uint32_t GetLastUptime();

        void SetThreshold(uint32_t threshold);

      protected:
        /* 节点上一个心跳包的时间 */
        volatile uint64_t last_uptime_ = 0;
        /* 判断节点离线的时间 */
        uint32_t online_threshold_ = 500;
        /**
         * @brief 更新心跳包
         */
        void Heartbeat();
    };

}  // namespace driver
