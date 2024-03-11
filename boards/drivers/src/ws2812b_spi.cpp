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

#include <string.h>

namespace display {

    WS2812::WS2812(const ws2812_init_t& init) {
        spi_ = init.spi;
        led_num_ = init.led_num;
        high_level_ = init.high_level;
        low_level_ = init.low_level;
        led_data_ = new uint8_t[led_num_ * 24];
    }
    WS2812::~WS2812() {
        delete led_data_;
    }
    void WS2812::SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
        for (uint8_t i = 0; i < 8; i++) {
            led_data_[index * 24 + i] = (g & (0x80 >> i)) ? high_level_ : low_level_;
            led_data_[index * 24 + i + 8] = (r & (0x80 >> i)) ? high_level_ : low_level_;
            led_data_[index * 24 + i + 16] = (b & (0x80 >> i)) ? high_level_ : low_level_;
        }
    }
    void WS2812::SetColor(uint16_t index, uint32_t aRGB) {
        volatile uint8_t alpha;
        volatile uint16_t red, green, blue;

        alpha = (aRGB & 0xFF000000) >> 24;
        red = ((aRGB & 0x00FF0000) >> 16) * alpha / 255.0;
        green = ((aRGB & 0x0000FF00) >> 8) * alpha / 255.0;
        blue = ((aRGB & 0x000000FF) >> 0) * alpha / 255.0;
        SetColor(index, red, green, blue);
    }
    void WS2812::SetColor(uint8_t r, uint8_t g, uint8_t b) {
        for (uint16_t i = 0; i < led_num_; i++) {
            SetColor(i, r, g, b);
        }
    }
    void WS2812::SetColor(uint32_t aRGB) {
        for (uint16_t i = 0; i < led_num_; i++) {
            SetColor(i, aRGB);
        }
    }
    void WS2812::Update() {
        spi_->Transmit(led_data_, led_num_ * 24);
    }
    void WS2812::Clear() {
        memset(led_data_, low_level_, led_num_ * 24);
    }
}  // namespace display