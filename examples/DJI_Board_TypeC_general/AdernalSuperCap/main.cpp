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

#include "main.h"

#include <usart.h>

#include "bsp_can.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "AdernalSuperCap.h"

/*------------------------ 全局变量定义 ------------------------*/
// CAN实例
static bsp::CAN* can1 = nullptr;

// 超级电容反馈数据
static Adernal_Fb_Typedef supercap_feedback;
static Adernal_SafetyLevel_Typedef supercap_safety_level[8];
static bool supercap_ready = false; // 初始化状态
static bool new_ready = false;
static bool new_feedback = false;
static bool new_safety = false;


/*------------------------ CAN回调函数 ------------------------*/
// CAN接收到对应ID的数据后会调用对应的回调函数，通过回调函数，我们可以接收到数据，并解析数据，并执行相应的操作。
// 注意：不要在回调函数内部写print串口信息，一定要将状态传输到RM_RTOS_Default_Task中的While来判断是否有状态更新并输出调试信息，否则无法输出！
// 就绪帧回调，由于超级电控上电只发送一次，若控制器正在运行，那么请重启控制器，才能接收到就绪帧
// 就绪帧由超级电容控制器发送至主控
void SuperCap_ReadyCallback(const uint8_t data[], void* args) {
    UNUSED(args);
    // 假定好CAN接收端就绪的数据头，以便和接收到的数据头进行匹配
    Usr_CAN_RxHeader_Typedef rx_header;
    rx_header.ID = __ADERNAL_CANID_STDBY;
    rx_header.IDtype = USR_CAN_IDTYPE_STANDARD;
    rx_header.FrameType = USR_CAN_FRAMETYPE_DATA;
    rx_header.DataLength = 1;
    // 判断，将构造好的数据头传入解析函数，并进行比对，判断是否为就绪帧且已就绪
    uint8_t state = Adernal_StdbyFrame_Unpack(&rx_header, (uint8_t*)data);
    // 如果状态为就绪，则将supercap_ready设置为true，否则设置为false
    if (state == Adernal_StdbyState_OK) {
        supercap_ready = true;
        new_ready = true;
        //print("SuperCap Ready!\r\n");
    } else {
        supercap_ready = false;
        //print("SuperCap Not Ready!\r\n");
    }
}

// 反馈帧回调
// 反馈帧由超级电容控制器发送至主控，每隔500ms传输一帧
void SuperCap_FeedbackCallback(const uint8_t data[], void* args) {
    UNUSED(args);
    // 假定好CAN接收端反馈的数据头，以便和接收到的数据头进行匹配
    Usr_CAN_RxHeader_Typedef rx_header;
    rx_header.ID = __ADERNAL_CANID_FB;
    rx_header.IDtype = USR_CAN_IDTYPE_STANDARD;
    rx_header.FrameType = USR_CAN_FRAMETYPE_DATA;
    rx_header.DataLength = 6;
    // 这里除了要传入数据头和反馈数据，还需要传入一个反馈数据结构体，用于存储反馈数据
    if (Adernal_FbFrame_Unpack(&rx_header, (uint8_t*)data, &supercap_feedback) == Adernal_FbState_OK) {
        // print("SuperCap Voltage: %.2fV, Power: %.2fW, Work1: %d%%, Work2: %d%%\r\n",
        //       supercap_feedback.Voltage_NoESR,
        //       supercap_feedback.Power_Battery,
        //       supercap_feedback.Work_Sentry1,
        //       supercap_feedback.Work_Sentry2);
        // // 延时500ms，避免频繁发送数据
        // osDelay(500);
        new_feedback = true;
    }
}

// 安全提示帧回调
// 初始化时传输一次该帧，上述任意一个检测项安全等级变化时再次传输
void SuperCap_SafetyCallback(const uint8_t data[], void* args) {
    UNUSED(args);
    // 假定好CAN接收端安全数据头，以便和接收到的数据头进行匹配
    Usr_CAN_RxHeader_Typedef rx_header;
    rx_header.ID = __ADERNAL_CANID_SAFE;
    rx_header.IDtype = USR_CAN_IDTYPE_STANDARD;
    rx_header.FrameType = USR_CAN_FRAMETYPE_DATA;
    rx_header.DataLength = 8;
    // 这里除了要传入数据头和反馈数据，还需要传入一个反馈数据结构体，用于存储反馈数据
    if (Adernal_Safety_Unpack(&rx_header, (uint8_t*)data, supercap_safety_level) == Adernal_SafetyState_OK) {
        // print("SuperCap Safety Levels: %d %d %d %d %d %d %d %d\r\n",
        //       supercap_safety_level[0], supercap_safety_level[1],
        //       supercap_safety_level[2], supercap_safety_level[3],
        //       supercap_safety_level[4], supercap_safety_level[5],
        //       supercap_safety_level[6], supercap_safety_level[7]);
        // osDelay(500);
        new_safety = true;
    }
}

/*------------------------ 初始化函数 ------------------------*/
// 如果DEBUG时程序未进入到这里，而是进入到Error_Handler，请尝试重置MCU
// 请不要在RM_RTOS_Init中尝试DEBUG一步一步输出调试信息，只会输出第一行，点击继续后才能正常输出！
// 如果有任何需要单步调试输出串口信息的，请移步至RM_RTOS_Default_task中的while循环！
void RM_RTOS_Init(void) {
    // 设置串口打印
    print_use_uart(&BOARD_UART2, true, 961200);
    print("Initializing... \r\n");

    // 初始化CAN
    can1 = new bsp::CAN(&hcan1, true);
    // 注册CAN回调函数
    can1->RegisterRxCallback(__ADERNAL_CANID_STDBY, SuperCap_ReadyCallback);
    can1->RegisterRxCallback(__ADERNAL_CANID_FB, SuperCap_FeedbackCallback);
    can1->RegisterRxCallback(__ADERNAL_CANID_SAFE, SuperCap_SafetyCallback);

    print("CAN initialized and callbacks registered\r\n");
}

/*---------------------- 主任务函数 ----------------------*/
void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    print("Waiting for SuperCap to be ready...\r\n");

    // 若控制器未启动，等待超级电容就绪
    //supercap_ready = true; //TODO DEBUG ONLY
    while (!supercap_ready) {
        print("Waiting...\r\n");
        osDelay(100);
    }

    // 初始化超级电容 - 选择类型1 (24V)
    if (Adernal_Init(Adernal_Init_CapType1) == Adernal_Tx_OK) {
        print("SuperCap initialized with Type 1 (24V)\r\n");
    } else {
        print("Failed to initialize SuperCap\r\n");
    }
    osDelay(500);
    // 注意，在设定模式之前，应检查并保证电池电压处于 19.5V 与 27.5V 之间，电容组电压处于3.5V与用户设定的电容组最高电压之间，否则将无法切换模式！

    // 设置预期功率为50W，Work模式，不开启Exceed
    Adernal_Ctrl_Typedef ctrl_config;
    ctrl_config.ExpectPwr = 50;  // 50W
    ctrl_config.Mode = Adernal_CtrlMode_Work;
    ctrl_config.Exceed = Adernal_CtrlExceed_Off;

    if (Adernal_Ctrl(&ctrl_config) == Adernal_Tx_OK) {
        print("Set Expect Power to 50W, Work mode, Exceed OFF\r\n");
    } else {
        print("Failed to set SuperCap parameters\r\n");
    }

    while(1) {
        // 处理就绪状态打印
        if (new_ready) {
            print("SuperCap Ready!\r\n");
            new_ready = false;
        }

        // 处理反馈数据打印
        if (new_feedback) {
            print("Voltage: %.2fV Power: %.2fW Work1:%d%% Work2:%d%%\r\n",
                  supercap_feedback.Voltage_NoESR,
                  supercap_feedback.Power_Battery,
                  supercap_feedback.Work_Sentry1,
                  supercap_feedback.Work_Sentry2);
            new_feedback = false;
            osDelay(500);
        }

        // 处理安全等级打印
        if (new_safety) {
            print("Safety Levels: %d %d %d %d %d %d %d %d\r\n",
                  supercap_safety_level[0], supercap_safety_level[1],
                  supercap_safety_level[2], supercap_safety_level[3],
                  supercap_safety_level[4], supercap_safety_level[5],
                  supercap_safety_level[6], supercap_safety_level[7]);
            new_safety = false;
        }

        osDelay(100); // 保持原有延时
    }

}