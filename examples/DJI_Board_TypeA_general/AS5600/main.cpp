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

#include <AS5600.h>

#include "MPU6500.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_spi.h"
#include "bsp_i2c.h"
#include "cmsis_os.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

static imu::AS5600* as5600 = nullptr;
static bsp::I2C* i2c = nullptr;


void RM_RTOS_Init(void) {
    bsp::i2c_init_t i2c_init = {
        .hi2c = &hi2c2,
        .mode = bsp::I2C_MODE_BLOCKING
    };
    i2c = new bsp::I2C(i2c_init);

    imu::as5600_init_t imu_init = {
        .hi2c = i2c,
        .i2c_addr = 0x36,
    };
    as5600 = new imu::AS5600(imu_init);
    as5600->Init();
    print_use_uart(&huart8, true, 921600);

}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    print("IMU Initialized!\r\n");
    osDelay(300);
    uint32_t ts;

    while (true) {
        as5600->Update();
        ts = HAL_GetTick();
        set_cursor(0, 0);
        clear_screen();
        print("Temp: %d\r\n", as5600->GetAngle());
        print("Temp: %d\r\n", as5600->GetRawAngle());
        print("[%lu ms]\r\n", (unsigned long)ts);
        osDelay(100);
    }
}
