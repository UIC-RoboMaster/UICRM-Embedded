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
#include "power_meter.h"

power_meter::power_meter(UART_HandleTypeDef* huart, float ratio, user_callback_t callback)
{
    this->huart = huart;

    uart = new bsp::UART(huart);
    uart->SetupRx(32);
    uart->RegisterCallback(uart_callback, this);

    filter_ratio = ratio;
    user_callback = callback;
}

void power_meter::uart_callback(void *args)
{
    if (args == nullptr)
        return;
    auto obj = (power_meter *)args;
    obj->callback();
}

void power_meter::callback()
{
    // Read data from uart
    uint8_t *buffer_p;
    uint8_t len = uart->Read<true>(&buffer_p);
    if (len != sizeof(power_meter_data_t))
        return;

    // Parsed as power_meter_data_t
    auto *data = (power_meter_data_t *)buffer_p;
    if (data->header != 0xC8C8 || data->tail != 0x8C8C)
        return;

    // Heartbeat
    ConnectionDriver::Heartbeat();

    // Apply filter
    voltage = (uint16_t)(data->voltage * filter_ratio + voltage * (1 - filter_ratio));
    current = (int16_t)(data->current * filter_ratio + current * (1 - filter_ratio));

    // Call user callback
    if (user_callback != nullptr)
        user_callback(voltage, current);
}