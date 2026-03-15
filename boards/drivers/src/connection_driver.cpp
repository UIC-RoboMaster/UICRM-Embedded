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

#include "connection_driver.h"

#include "bsp_os.h"
namespace driver {
    bool ConnectionDriver::IsOnline() const {
        if (last_uptime_ == 0) {
            return false;
        }
        return bsp::GetHighresTickMilliSec() - last_uptime_ < online_threshold_;
    }
    void ConnectionDriver::Heartbeat() {
        last_uptime_ = bsp::GetHighresTickMilliSec();
        last_uptime_microsec_ = bsp::GetHighresTickMicroSec();
    }
    uint32_t ConnectionDriver::GetLastUptime() {
        return last_uptime_;
    }
    uint32_t ConnectionDriver::GetLastUptimeMicrosec() {
        return last_uptime_microsec_;
    }
    void ConnectionDriver::SetThreshold(uint32_t threshold) {
        online_threshold_ = threshold;
    }
}  // namespace driver
