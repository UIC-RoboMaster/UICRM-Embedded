#pragma once

#include <cinttypes>

#include "usart.h"

/**
 * @brief use a uart port for debug print
 *
 * @param huart HAL uart handle
 */
void print_use_uart(UART_HandleTypeDef *huart);

/**
 * @brief use USB virtual com port for debug print
 */
void print_use_usb();

/**
 * @brief print debug message via USB-OTG-FS
 *
 * @param format  formatted string
 * @param ...     same argument lists as in printf
 *
 * @return  number of bytes printed
 *
 * @note    this function requires sufficient stack allocation
 * @note    maximum print length is 32
 * @note    will perform no-op in NDEBUG mode
 */
int32_t print(const char *format, ...);

/* escape codes helper functions -- http://www.termsys.demon.co.uk/vtansi.htm */

/**
 * @brief set the cursor with escape codes
 *
 * @param row row of the cursor
 * @param col column of the cursor
 */
void set_cursor(int row, int col);

/**
 * @brief clear uart screen
 */
void clear_screen(void);
