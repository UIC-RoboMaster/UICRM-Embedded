#include "bsp_gpio.h"

#include "FreeRTOS.h"

namespace bsp {

GPIO::GPIO(GPIO_TypeDef *group, uint16_t pin)
    : group_(group), pin_(pin), state_(0) {}

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

GPIT *GPIT::gpits[NUM_GPITS] = {NULL};

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
