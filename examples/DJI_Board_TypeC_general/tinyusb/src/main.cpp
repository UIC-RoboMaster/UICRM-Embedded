/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "bsp/board_api.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

// static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
// static bool blink_enable = true;

// void led_blinking_task(void);
void cdc_task(void);

/*------------- MAIN -------------*/

void RM_RTOS_Init(void) {
    print_use_uart(&huart1);
    // ---- USB OTG FS 硬件初始化 ----
    // 1. USB 引脚配置 (PA11=DM, PA12=DP)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 2. 开 USB OTG FS 时钟
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    // 3. 配置 USB 中断优先级（FreeRTOS 下必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    // ---- TinyUSB 初始化 ----
    tud_configure_dwc2_t cfg = CFG_TUD_CONFIGURE_DWC2_DEFAULT;
    cfg.vbus_sensing = 1;
    tud_configure(BOARD_TUD_RHPORT, TUD_CFGID_DWC2, &cfg);
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    while (true) {
        // set_cursor(0, 0);
        // clear_screen();

        // osDelay(50);
        tud_task();  // tinyusb device task
        // led_blinking_task();

        cdc_task();
    }
}

// int main(void) {
//   board_init();
//
//   // init device stack on configured roothub port
//   tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
//   tusb_init(BOARD_TUD_RHPORT, &dev_init);
//
//   board_init_after_tusb();
//
//   while (1) {
//     tud_task(); // tinyusb device task
//     led_blinking_task();
//
//     cdc_task();
//   }
// }

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
    // blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    // blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    // blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    // blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void) {
    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if ( tud_cdc_connected() )
    {
        // connected and there are data available
        if (tud_cdc_available()) {
            // read data
            char buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf));
            (void)count;

            // Echo back
            // Note: Skip echo by commenting out write() and write_flush()
            // for throughput test e.g
            //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
            tud_cdc_write(buf, count);
            tud_cdc_write_flush();
        }

        // Press on-board button to send Uart status notification
        // static cdc_notify_uart_state_t uart_state = {.value = 0};

        // static uint32_t btn_prev = 0;
        // const uint32_t  btn      = board_button_read();
        //
        //     if ((btn_prev == 0u) && (btn != 0u)) {
        //       uart_state.dsr ^= 1;
        //       uart_state.dcd ^= 1;
        //       tud_cdc_notify_uart_state(&uart_state);
        //     }
        //     btn_prev = btn;
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;
    (void)dtr;

    //   if (dtr) {
    //     // Terminal connected
    //     blink_enable = false;
    //     board_led_write(true);
    //   } else {
    //     // Terminal disconnected
    //     blink_enable = true;
    //   }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
    (void) max_len;
    volatile uint32_t *stm32_uuid = (volatile uint32_t *) UID_BASE;
    uint32_t *id32 = (uint32_t *) (uintptr_t) id;
    uint8_t const len = 12;

    id32[0] = stm32_uuid[0];
    id32[1] = stm32_uuid[1];
    id32[2] = stm32_uuid[2];

    return len;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
// void led_blinking_task(void) {
//   static uint32_t start_ms  = 0;
//   static bool     led_state = false;

// if (blink_enable) {
//   // Blink every interval ms
//   if (tusb_time_millis_api() - start_ms < blink_interval_ms) {
//     return; // not enough time
//   }
//   start_ms += blink_interval_ms;

// board_led_write(led_state);
// led_state = !led_state;
// }
// }
