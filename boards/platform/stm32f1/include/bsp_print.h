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

#pragma once

#include <cinttypes>

#include "bsp_uart.h"
#include "usart.h"

/**
 * @brief 使用串口进行调试打印
 *
 * @param huart HAL uart 句柄
 * @param dma 是否使用DMA
 */
/**
 * @brief use a uart port for debug print
 *
 * @param huart HAL uart handle
 * @param dma whether to use DMA
 */
void print_use_uart(UART_HandleTypeDef* huart, bool dma = true, uint32_t baudrate = 115200);

#ifndef NO_USB
/**
 * @brief 使用USB虚拟串口进行调试打印
 */
/**
 * @brief use USB virtual com port for debug print
 */
void print_use_usb();
#endif

/**
 * @brief dump raw array data to uart or usb
 */
uint32_t dump(const void* data, uint8_t length);

/**
 * @brief print data in a pretty format, representing hexadecimal numbers in text
 */
void dump_pretty(const void* data, uint8_t length);

/**
 * @brief 输出数据，类似于 printf
 *
 * @param format 格式化字符串
 * @param ...    与 printf 相同的参数列表
 *
 * @return 打印的字节数
 *
 * @note    此函数需要足够的栈空间
 * @note    最大打印长度为 32
 * @note    会在 NDEBUG 模式下不执行任何操作
 */
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
uint32_t print(const char* format, ...);

/* escape codes helper functions -- http://www.termsys.demon.co.uk/vtansi.htm
 */

/**
 * @brief 设置光标位置
 *
 * @param row 光标所在行
 * @param col 光标所在列
 */
/**
 * @brief set the cursor with escape codes
 *
 * @param row row of the cursor
 * @param col column of the cursor
 */
void set_cursor(int row, int col);

/**
 * @brief 清空屏幕
 */
/**
 * @brief clear uart screen
 */
void clear_screen(void);
