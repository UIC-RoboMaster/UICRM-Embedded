// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Sarzn on 2026/4/14.
//

#include "main.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "../include/config.h"

class RefereeUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    void RxCompleteCallback() final {};
};

static bsp::GPIO* led = nullptr;
static RefereeUART* referee_uart = nullptr;
static communication::Referee* referee = nullptr;
static bsp::GPIO* key_50 = nullptr;
static bsp::GPIO* key_100 = nullptr;
static bsp::GPIO* key_200 = nullptr;
static bsp::GPIO* sw_left = nullptr;  // 拨动开关(上拉)，往左拨是 O(17mm 弹丸)，往右拨是 I(42mm 弹丸)
static bsp::GPIO* sw_right = nullptr;
static bool is_left = true;
static osSemaphoreId_t led_sem = nullptr;


/**
 *@brief 一次鼠标点击
 *
 *一次鼠标点击的过程包括三个
 *1.移动到 x-y 设定的位置 2.按下左键 3.松开左键
 *每个操作都要用 Transmit 发一次包
 *
 * @param x
 * @param y
 */
static void send_click(uint16_t x, uint16_t y) {
    for (int i = 1; i <= 3; i++) {
        if (i == 1)
            referee->custom_client_data = {0, x, 0, y, 0, 0};
        else if (i == 2)
            referee->custom_client_data.mouse_left = 1;
        else if (i == 3)
            referee->custom_client_data.mouse_left = 0;

        referee->Transmit(communication::CUSTOM_CLIENT_DATA);  // Transmit 发包
        osDelay(CLICK_DELAY_MS);
    }
}

/**
 * 按下 O/I 键（买弹）
 * 根据拨动开关的位置
 */
static void click_OI() {
    uint16_t key = is_left ? (uint16_t)'O' : (uint16_t)'I';
    referee->custom_client_data = {key, 0, 0, 0, 0, 0};
    referee->Transmit(communication::CUSTOM_CLIENT_DATA);
    osDelay(CLICK_DELAY_MS);
}

/**
 * @brief 完整的一键买弹流程
 * @param x
 * @param y
 * @param double_click
 */
static void buy_bullets(uint16_t x, uint16_t y, bool double_click) {
    // 买弹
    click_OI();
    send_click(x, y);
    if (double_click)
        send_click(x, y);
    // 确认
    send_click(BUY_X, BUY_Y);
    send_click(CONFIRM_BUY_X, CONFIRM_BUY_Y);
    // 归位
    click_OI();
    referee->custom_client_data = {0, 0, 0, 0, 0, 0};
    referee->Transmit(communication::CUSTOM_CLIENT_DATA);
    osDelay(CLICK_DELAY_MS);
}

void RM_RTOS_Init(void) {
    // USART1-PA9/10
    referee_uart = new RefereeUART(&huart1);
    referee_uart->SetupRx(300);
    referee_uart->SetupTx(300);
    referee = new communication::Referee(referee_uart);
    // GPIO 使能
    led     = new bsp::GPIO(LED_GPIO_Port,     LED_Pin);
    key_50  = new bsp::GPIO(KEY_50_GPIO_Port,  KEY_50_Pin);
    key_100 = new bsp::GPIO(KEY_100_GPIO_Port, KEY_100_Pin);
    key_200 = new bsp::GPIO(KEY_200_GPIO_Port, KEY_200_Pin);
    sw_left  = new bsp::GPIO(SW_LEFT_GPIO_Port,  SW_LEFT_Pin);
    sw_right = new bsp::GPIO(SW_RIGHT_GPIO_Port, SW_RIGHT_Pin);
    // 创建二值信号量，初始计数为 0
    led_sem = osSemaphoreNew(1, 0, nullptr);
}

// LED 线程的属性配置
static const osThreadAttr_t led_task_attr = {
    .name       = "ledTask",
    .attr_bits  = 0U,           // 显式设为 0
    .cb_mem     = nullptr,
    .cb_size    = 0U,
    .stack_mem  = nullptr,
    .stack_size = 128 * 4,  // 128 words
    .priority   = osPriorityLow,  // 低优先级
    .tz_module  = 0U,
    .reserved   = 0U,
};
// LED 线程函数
static void led_task(void* arg) {
    UNUSED(arg);
    while (true) {
        // 阻塞等待信号，永不超时
        osSemaphoreAcquire(led_sem, osWaitForever);

        // 收到信号后独立完成闪烁
        led->High();
        osDelay(80);
        led->Low();
        osDelay(80);
        led->High();
        osDelay(80);
        led->Low();
    }
}

void RM_RTOS_Threads_Init(void) {
    osThreadNew(led_task, nullptr, &led_task_attr);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    // 小登快闪-初始化
    for (int i = 0; i < 3; i++) {
        osSemaphoreRelease(led_sem);
    }
    bool flag50 = false, flag100 = false, flag200 = false;

    while (true) {
        // 拨动开关-位置判断
        bool left_pressed = (sw_left->Read() == 0);  // 低电平有效
        bool right_pressed = (sw_right->Read() == 0);

        if (left_pressed && !right_pressed)
            is_left = true;  // 拨到左边-17mm
        else if (!left_pressed && right_pressed)
            is_left = false;  // 拨到右边-42mm
        // 中间位置(如果有的话)-保持上次状态

        // 按键
        bool k50 = (key_50->Read() == 0);
        bool k100 = (key_100->Read() == 0);
        bool k200 = (key_200->Read() == 0);

        // 防止长按连点
        if (!k50)
            flag50 = false;
        if (!k100)
            flag100 = false;
        if (!k200)
            flag200 = false;

        // 下降沿
        if (k50 && !flag50) {
            buy_bullets(POS_50_X, POS_50_Y, false);
            flag50 = true;
            // const char* msg = "hello 50\r\n";
            // referee_uart->Write((uint8_t*)msg, strlen(msg));
            osSemaphoreRelease(led_sem);
        }
        if (k100 && !flag100) {
            buy_bullets(POS_100_X, POS_100_Y, false);
            flag100 = true;
            // const char* msg = "hello 100\r\n";
            // referee_uart->Write((uint8_t*)msg, strlen(msg));
            osSemaphoreRelease(led_sem);
        }
        if (k200 && !flag200) {
            buy_bullets(POS_200_X, POS_200_Y, true);
            flag200 = true;
            // const char* msg = "hello 200\r\n";
            // referee_uart->Write((uint8_t*)msg, strlen(msg));
            osSemaphoreRelease(led_sem);
        }
        osDelay(LOOP_DELAY_MS);
    }
}