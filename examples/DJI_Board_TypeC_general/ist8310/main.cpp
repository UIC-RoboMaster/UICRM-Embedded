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

// clang-format off
#include "IST8310.h"
// clang-format on

static bsp::I2C* i2c3 = nullptr;
static imu::IST8310* ist8310 = nullptr;
static bsp::GPIO* ist8310_rst = nullptr;
static bsp::GPIT* ist8310_int = nullptr;

void RM_RTOS_Init(void) {
    HAL_Delay(500);
    print_use_uart(&huart1);
    HAL_Delay(500);
    print("IST8310 Test\r\n");
    ist8310_rst = new bsp::GPIO(GPIOG, GPIO_PIN_6);
    ist8310_int = new bsp::GPIT(DRDY_IST8310_Pin);
    bsp::i2c_init_t i2c3_init = {.hi2c = &hi2c3, .mode = bsp::I2C_MODE_DMA};
    i2c3 = new bsp::I2C(i2c3_init);
    imu::IST8310_init_t ist8310Init = {.i2c = i2c3, .drdy = ist8310_int, .rst = ist8310_rst};
    ist8310 = new imu::IST8310(ist8310Init);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (!ist8310->IsReady()) {
        osDelay(50);
    }
    while (true) {
        clear_screen();
        set_cursor(0, 0);
        float x, y, z;
        x = ist8310->mag_[0];
        y = ist8310->mag_[1];
        z = ist8310->mag_[2];
        print("MAG: %.2f %.2f %.2f  \r\n", x, y, z);
        osDelay(200);
    }
}
