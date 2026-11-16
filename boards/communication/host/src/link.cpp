/*###########################################################
# Copyright (c) 2026-2027. BNBU/UIC RoboMaster             #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS FOR   #
 # A PARTICULAR PURPOSE.  See the GNU General Public License#
 # for more details.                                        #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "link.h"

#include "bsp_uart.h"
#ifndef NO_USB
#include "bsp_usb.h"
#endif

namespace host_comm {

    void BspLink::SetRxHandler(RxHandler handler, void* ctx) {
        handler_ = handler;
        ctx_ = ctx;
    }

    void BspLink::Callback(void* args) {
        BspLink* self = static_cast<BspLink*>(args);
        if (self->handler_ != nullptr && self->rx_length_ > 0)
            self->handler_(self->ctx_, self->rx_data_, self->rx_length_);
    }

    UartLink::UartLink(bsp::UART* uart) : uart_(uart) {
        uart_->SetupRxData(&rx_data_, &rx_length_);
        uart_->RegisterCallback(Callback, this);
    }

    int UartLink::Write(const uint8_t* data, size_t size) {
        return uart_->Write(data, static_cast<uint32_t>(size));
    }

#ifndef NO_USB
    // 定义在 usb_device.c：未枚举完成时 pClassData 仍为空，
    // CDC_Transmit_FS 会直接解引用它，必须在外面先挡掉。
    extern "C" USBD_HandleTypeDef hUsbDeviceFS;

    UsbLink::UsbLink(bsp::VirtualUSB* usb) : usb_(usb) {
        usb_->SetupRxData(&rx_data_, &rx_length_);
        usb_->RegisterCallback(Callback, this);
    }

    int UsbLink::Write(const uint8_t* data, size_t size) {
        // 主机还没配置设备时不要进 CDC 栈，返回 0 让 Host 下一 tick 重试。
        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
            return 0;
        // bsp 接口未加 const，语义上不修改数据。
        return static_cast<int>(usb_->Write(const_cast<uint8_t*>(data), static_cast<uint32_t>(size)));
    }
#endif

}  // namespace host_comm
