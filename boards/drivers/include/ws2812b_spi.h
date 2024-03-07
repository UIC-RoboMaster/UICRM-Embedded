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
#include "bsp_spi.h"

/* The High and Low output for SPI with 6M Clock */
#define WS2812_SPI_6M_LowLevel 0xC0   // Low
#define WS2812_SPI_6M_HighLevel 0xF0  // High

/* The High and Low output for SPI with 5.25M Clock */
#define WS2812_SPI_5M25_LowLevel 0xC0   // Low
#define WS2812_SPI_5M25_HighLevel 0xFC  // High

namespace display {
    struct ws2812_init_t {
        bsp::SPI* spi;
        uint16_t led_num;
        uint8_t high_level;
        uint8_t low_level;
    };

    class WS2812 {
      public:
        WS2812(const ws2812_init_t& init);
        ~WS2812();
        void SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
        void SetColor(uint16_t index, uint32_t aRGB);
        void SetColor(uint8_t r, uint8_t g, uint8_t b);
        void SetColor(uint32_t aRGB);
        void Update();
        void Clear();

      private:
        bsp::SPI* spi_;
        uint16_t led_num_;
        uint8_t* led_data_;
        uint8_t high_level_;
        uint8_t low_level_;
    };
}  // namespace driver