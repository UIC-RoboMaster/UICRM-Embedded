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

#include "oled.h"

#include <cstdarg>
#include <cstdio>

#include "oled_fonts/ascii.h"

namespace display {

    OLED::OLED(bsp::I2C* i2c, uint16_t OLED_i2c_addr) {
        i2c_ = i2c;
        OLED_i2c_addr_ = OLED_i2c_addr;
        Init();
    }

    bool OLED::IsReady() {
        return i2c_->isReady(OLED_i2c_addr_);
    }

    void OLED::WriteByte(uint8_t dat, uint8_t cmd) {
        uint8_t cmd_data[2];
        if (cmd == OLED_CMD)
            cmd_data[0] = 0x00;
        else
            cmd_data[0] = 0x40;
        cmd_data[1] = dat;
        i2c_->Transmit(OLED_i2c_addr_, cmd_data, 2);
    }

    void OLED::Init() {
        WriteByte(0xAE, OLED_CMD);  // display off
        WriteByte(0x20, OLED_CMD);  // Set Memory Addressing Mode
        WriteByte(0x10,
                  OLED_CMD);  // 00,Horizontal Addressing Mode;01,Vertical Addressing
                              // Mode;10,Page Addressing Mode (RESET);11,Invalid
        WriteByte(0xb0,
                  OLED_CMD);        // Set Page Start Address for Page Addressing Mode,0-7
        WriteByte(0xc8, OLED_CMD);  // Set COM Output Scan Direction
        WriteByte(0x00, OLED_CMD);  //---set low column address
        WriteByte(0x10, OLED_CMD);  //---set high column address
        WriteByte(0x40, OLED_CMD);  //--set start line address
        WriteByte(0x81, OLED_CMD);  //--set contrast control register
        WriteByte(0xff, OLED_CMD);  // brightness 0x00~0xff
        WriteByte(0xa1, OLED_CMD);  //--set segment re-map 0 to 127
        WriteByte(0xa6, OLED_CMD);  //--set normal display
        WriteByte(0xa8, OLED_CMD);  //--set multiplex ratio(1 to 64)
        WriteByte(0x3F, OLED_CMD);  //
        WriteByte(0xa4, OLED_CMD);  // 0xa4,Output follows RAM content;0xa5,Output
                                    // ignores RAM content
        WriteByte(0xd3, OLED_CMD);  //-set display offset
        WriteByte(0x00, OLED_CMD);  //-not offset
        WriteByte(0xd5,
                  OLED_CMD);        //--set display clock divide ratio/oscillator frequency
        WriteByte(0xf0, OLED_CMD);  //--set divide ratio
        WriteByte(0xd9, OLED_CMD);  //--set pre-charge period
        WriteByte(0x22, OLED_CMD);  //
        WriteByte(0xda, OLED_CMD);  //--set com pins hardware configuration
        WriteByte(0x12, OLED_CMD);
        WriteByte(0xdb, OLED_CMD);  //--set vcomh
        WriteByte(0x20, OLED_CMD);  // 0x20,0.77xVcc
        WriteByte(0x8d, OLED_CMD);  //--set DC-DC enable
        WriteByte(0x14, OLED_CMD);  //
        WriteByte(0xaf, OLED_CMD);  //--turn on oled panel
    }

    void OLED::DisplayOn() {
        WriteByte(0x8d, OLED_CMD);
        WriteByte(0x14, OLED_CMD);
        WriteByte(0xaf, OLED_CMD);
    }

    void OLED::DisplayOff() {
        WriteByte(0x8d, OLED_CMD);
        WriteByte(0x10, OLED_CMD);
        WriteByte(0xae, OLED_CMD);
    }

    void OLED::OperateGram(pen_typedef pen) {
        for (int i = 0; i < 8; ++i) {
            for (int n = 0; n < 128; ++n) {
                if (pen == PEN_WRITE) {
                    OLED_GRAM_[n][i] = 0xff;
                } else if (pen == PEN_CLEAR) {
                    OLED_GRAM_[n][i] = 0x00;
                } else {
                    OLED_GRAM_[n][i] = 0xff - OLED_GRAM_[n][i];
                }
            }
        }
    }

    void OLED::SetPos(uint8_t x, uint8_t y) {
        WriteByte((0xb0 + y), OLED_CMD);                // set page address y
        WriteByte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);  // set column high address
        WriteByte((x & 0x0f), OLED_CMD);
    }

    void OLED::DrawPoint(int8_t x, int8_t y, pen_typedef pen) {
        /* check the corrdinate */
        if ((x < 0) || (x > (OLED_X_WIDTH - 1)) || (y < 0) || (y > (OLED_Y_WIDTH - 1)))
            return;
        uint8_t page = y / 8, row = y % 8;

        if (pen == PEN_WRITE) {
            OLED_GRAM_[x][page] |= 1 << row;
        } else if (pen == PEN_INVERSION) {
            OLED_GRAM_[x][page] ^= 1 << row;
        } else {
            OLED_GRAM_[x][page] &= ~(1 << row);
        }
    }

    void OLED::DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen) {
        uint8_t col, row;
        uint8_t x_st, x_ed, y_st, y_ed;
        float k, b;

        if (y1 == y2) {
            (x1 <= x2) ? (x_st = x1) : (x_st = x2);
            (x1 <= x2) ? (x_ed = x2) : (x_ed = x1);

            for (col = x_st; col <= x_ed; ++col) {
                DrawPoint(col, y1, pen);
            }
        } else if (x1 == x2) {
            (y1 <= y2) ? (y_st = y1) : (y_st = y2);
            (y1 <= y2) ? (y_ed = y2) : (y_ed = y1);

            for (row = y_st; row <= y_ed; ++row) {
                DrawPoint(x1, row, pen);
            }
        } else {
            k = ((float)(y2 - y1)) / (x2 - x1);
            b = (float)y1 - k * x1;

            (x1 <= x2) ? (x_st = x1) : (x_st = x2);
            (x1 <= x2) ? (x_ed = x2) : (x_ed = x1);

            for (col = x_st; col <= x_ed; ++col) {
                DrawPoint(col, (uint8_t)(col * k + b), pen);
            }
        }
    }

    void OLED::ShowChar(uint8_t row, uint8_t col, uint8_t chr) {
        uint8_t x = col * 6;
        uint8_t y = row * 12;
        uint8_t temp, t, t1;
        uint8_t y0 = y;
        chr = chr - ' ';

        for (t = 0; t < 12; ++t) {
            temp = asc2_1206[chr][t];
            for (t1 = 0; t1 < 8; ++t1) {
                if (temp & 0x80)
                    DrawPoint(x, y, PEN_WRITE);
                else
                    DrawPoint(x, y, PEN_CLEAR);
                temp <<= 1;
                y++;
                if ((y - y0) == 12) {
                    y = y0;
                    x++;
                    break;
                }
            }
        }
    }

    const unsigned char block_graph[4][12] = {
        {0x00, 0x00, 0x7F, 0xE0, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20}, /*wrong block*/
        {0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x7F, 0xE0, 0x00, 0x00}, /*wrong block*/
        {0x00, 0x00, 0x7F, 0xE0, 0x40, 0x20, 0x42, 0x20, 0x41, 0x20, 0x40, 0xA0}, /*correct block*/
        {0x43, 0x20, 0x4C, 0x20, 0x58, 0x20, 0x40, 0x20, 0x7F, 0xE0, 0x00, 0x00}, /*correct block*/
    };

    void OLED::ShowBlock(uint8_t row, uint8_t col, bool correct) {
        uint8_t x = col * 6;
        uint8_t y = row * 12;
        uint8_t temp, t, t1;
        uint8_t y0 = y;

        for (t = 0; t < 12; ++t) {
            temp = block_graph[2 * correct][t];
            for (t1 = 0; t1 < 8; ++t1) {
                if (temp & 0x80)
                    DrawPoint(x, y, PEN_WRITE);
                else
                    DrawPoint(x, y, PEN_CLEAR);
                temp <<= 1;
                y++;
                if ((y - y0) == 12) {
                    y = y0;
                    x++;
                    break;
                }
            }
        }

        x = (col + 1) * 6;
        y = row * 12;
        y0 = y;

        for (t = 0; t < 12; ++t) {
            temp = block_graph[2 * correct + 1][t];
            for (t1 = 0; t1 < 8; ++t1) {
                if (temp & 0x80)
                    DrawPoint(x, y, PEN_WRITE);
                else
                    DrawPoint(x, y, PEN_CLEAR);
                temp <<= 1;
                y++;
                if ((y - y0) == 12) {
                    y = y0;
                    x++;
                    break;
                }
            }
        }
    }

    void OLED::ShowString(uint8_t row, uint8_t col, uint8_t* chr) {
        uint8_t n = 0;

        while (chr[n] != '\0') {
            if (chr[n] == '\n') {
                col = 0;
                row += 1;
                ++n;
                continue;
            }
            ShowChar(row, col, chr[n]);
            ++col;
            if (col > 20) {
                col = 0;
                row += 1;
            }
            ++n;
        }
    }

    void OLED::Printf(uint8_t row, uint8_t col, const char* fmt, ...) {
        uint8_t LCD_BUF[128] = {0};
        va_list ap;
        uint8_t remain_size;

        if (row > 4 || col > 20)
            return;
        va_start(ap, fmt);
        vsprintf((char*)LCD_BUF, fmt, ap);
        va_end(ap);
        remain_size = 21 - col;
        LCD_BUF[remain_size] = '\0';
        ShowString(row, col, LCD_BUF);
    }

    void OLED::RefreshGram() {
        uint8_t i, n;

        for (i = 0; i < 8; ++i) {
            SetPos(0, i);
            for (n = 0; n < 128; ++n) {
                WriteByte(OLED_GRAM_[n][i], OLED_DATA);
            }
        }
    }

    void OLED::ShowPic(const picture_t& pic, int8_t row, int8_t col, bool clear) {
        uint8_t temp_char;
        if (clear)
            OperateGram(PEN_CLEAR);
        for (uint8_t y = 0; y < pic.height; y += 8) {
            for (uint8_t x = 0; x < pic.width; ++x) {
                temp_char = pic.data[x][y / 8];
                for (int i = 0; i < 8; ++i) {
                    if (temp_char & 0x80) {
                        DrawPoint(x + col, y + i + row, PEN_WRITE);
                    } else {
                        DrawPoint(x + col, y + i + row, PEN_CLEAR);
                    }
                    temp_char <<= 1;
                }
            }
        }
        RefreshGram();
    }

}  // namespace display