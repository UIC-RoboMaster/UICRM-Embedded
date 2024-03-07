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

#include "ws2812b_spi.h"

namespace driver {

    WS2812::WS2812(const ws2812_init_t& init){
        spi_ = init.spi;
        led_num_ = init.led_num;
        high_level_ = init.high_level;
        low_level_ = init.low_level;
        led_data_ = new uint8_t[led_num_ * 24];
    }
}