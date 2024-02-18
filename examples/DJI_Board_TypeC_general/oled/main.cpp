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

#include "main.h"

#include "bsp_i2c.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "oled.h"
#include "oled_fonts/cat.h"
#include "oled_fonts/navigator.h"
#include "oled_fonts/robomaster.h"

static bsp::I2C* i2c2 = nullptr;
static display::OLED* OLED = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);
    bsp::i2c_init_t i2c2_init = {.hi2c = &hi2c2, .mode = bsp::I2C_MODE_BLOCKING};
    i2c2 = new bsp::I2C(i2c2_init);
    OLED = new display::OLED(i2c2, 0x78);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    OLED->ShowPic(display::RM_Logo);
    osDelay(2000);

    OLED->ShowPic(display::Navigator_Logo);
    osDelay(5000);

    uint8_t i = 0;
    while (true) {
        OLED->ShowPic(display::cat[i]);
        osDelay(1);
        i++;
        i = i % 12;
    }
}
