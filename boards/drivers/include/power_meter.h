
/*###########################################################
 # Copyright (c) 2025. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_uart.h"
#include "connection_driver.h"

/*
 * @note 适配7C制作的电流计
 */
class power_meter : public driver::ConnectionDriver
{
    // Packet data structure
    typedef struct
    {
        uint16_t header;  // 0xC8C8
        uint16_t voltage; // mV
        int16_t current;  // mA
        uint16_t tail;    // 0x8C8C
    } power_meter_data_t;

    // Callback type
    typedef void (*user_callback_t)(uint16_t voltage, int16_t current);

    UART_HandleTypeDef* huart;
    bsp::UART *uart;

    // Configs
    float filter_ratio;
    user_callback_t user_callback;

public:
    uint16_t voltage; // mV
    int16_t current;  // mA

public:
    /*
     * @param uart 串口对象
     * @param ratio 滤波系数，0~1之间，越小的值表示更好的滤波效果
     * @param callback 收到数据包后的用户回调函数（在中断中调用！）
     * @note 调用该构造函数初始化串口。串口收到数据时进入中断，在中断中处理数据，然后调用用户函数。
     */
    power_meter(UART_HandleTypeDef* huart, float ratio, user_callback_t callback);

    /*
     * @brief UART收到数据后调用，这个函数利用args找到是哪一个对象。
     */
    static void uart_callback(void *args);

private:
    /*
     * @brief 对象的回调函数，从UART读取数据并处理
     */
    void callback();
};