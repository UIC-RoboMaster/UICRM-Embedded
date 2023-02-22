#pragma once

/**
 * @note List of GPIOs that can be used on board now
 *    K1: Input,   K2: Output
 *    L1: Input,   L2: Output
 *    M1: Input,   M2: Output
 *    P1: Input,   P2: Output
 *    Q1: Input,   Q2: Output
 */

#include "bsp_error_handler.h"
#include "main.h"

#define NUM_GPITS 16

namespace bsp {

class GPIO {
public:
  /**
   * @brief constructor for generic GPIO (non interrupt)
   *
   * @param group GPIO group
   * @param pin   GPIO pin number
   */
  GPIO(GPIO_TypeDef *group, uint16_t pin);

  /**
   * @brief Set high output
   */
  void High();

  /**
   * @brief Set low output
   */
  void Low();

  /**
   * @brief Toggle GPIO output
   */
  void Toggle();

  /**
   * @brief Read GPIO input
   *
   * @return 1 for high and 0 for low
   */
  uint8_t Read();

private:
  GPIO_TypeDef *group_;
  uint16_t pin_;
  uint8_t state_;
};

class GPIT {
public:
  /**
   * @brief Contructor for general purpose interrupt pins
   *
   * @param pin interrupt pin number (x in EXTIx)
   */
  GPIT(uint16_t pin);

  /**
   * @brief Callback back when interrupt happens
   */
  virtual void IntCallback() = 0;

  /**
   * @brief wrapper for global interrupt handler
   *
   * @param pin interrupt pin number
   */
  static void IntCallback(uint16_t pin);

private:
  static int GetGPIOIndex(uint16_t pin);

  static GPIT *gpits[NUM_GPITS];

  uint16_t pin_;
};

} /* namespace bsp */
