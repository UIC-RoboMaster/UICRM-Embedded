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

#include "bsp_gpio.h"

#include "FreeRTOS.h"

namespace bsp {

    GPIO::GPIO(GPIO_TypeDef* group, uint16_t pin) : group_(group), pin_(pin), state_(0) {
    }

    /**
    * @brief 构造函数，用于需要自定义输入模式/上下拉的 GPIO（非中断）。
    *
    * 该构造函数允许在创建 GPIO 对象时直接配置引脚的模式（输入/输出）和上下拉电阻。
    * 通常用于按键、开关等需要上拉或下拉输入的场景。
    *
    * @param group GPIO 端口组（如 GPIOB、GPIOC）。
    * @param pin   GPIO 引脚号（如 GPIO_PIN_12）。
    * @param mode  引脚模式，可取 GPIO_MODE_INPUT、GPIO_MODE_OUTPUT_PP 等（HAL 定义）。
    * @param pull  上下拉配置，可取 GPIO_NOPULL、GPIO_PULLUP、GPIO_PULLDOWN。
    */
    GPIO::GPIO(GPIO_TypeDef* group, uint16_t pin, uint32_t mode, uint32_t pull)
    : group_(group), pin_(pin), state_(0) {
        GPIO_InitTypeDef init = {};
        init.Pin = pin;
        init.Mode = mode;        // GPIO_MODE_INPUT
        init.Pull = pull;        // GPIO_PULLUP
        init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(group, &init);
    }

    void GPIO::High() {
        HAL_GPIO_WritePin(group_, pin_, GPIO_PIN_SET);
        state_ = 1;
    }

    void GPIO::Low() {
        HAL_GPIO_WritePin(group_, pin_, GPIO_PIN_RESET);
        state_ = 0;
    }

    void GPIO::Toggle() {
        HAL_GPIO_TogglePin(group_, pin_);
        state_ ^= 1;
    }

    uint8_t GPIO::Read() {
        state_ = (HAL_GPIO_ReadPin(group_, pin_) == GPIO_PIN_SET);
        return state_;
    }

    GPIT* GPIT::gpits[NUM_GPITS] = {NULL};

    GPIT::GPIT(uint16_t pin) : pin_(pin) {
        int gpio_idx = GetGPIOIndex(pin);
        if (gpio_idx >= 0 && gpio_idx < NUM_GPITS && !gpits[gpio_idx])
            gpits[gpio_idx] = this;
    }

    void GPIT::IntCallback(uint16_t pin) {
        int gpio_idx = GetGPIOIndex(pin);
        if (gpio_idx >= 0 && gpio_idx < NUM_GPITS && gpits[gpio_idx])
            gpits[gpio_idx]->IntCallback();
    }

    int GPIT::GetGPIOIndex(uint16_t pin) {
        for (int i = 0; i < NUM_GPITS; ++i)
            if (pin == (1 << i))
                return i;
        return -1;
    }

} /* namespace bsp */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    bsp::GPIT::IntCallback(GPIO_Pin);
}
