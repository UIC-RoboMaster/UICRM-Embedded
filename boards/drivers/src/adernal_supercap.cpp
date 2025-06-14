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

#include "adernal_supercap.h"

/* USER Customize BEGIN CAN Library files */
#include "bsp_can.h"
namespace driver {
    /** @defgroup Adernal_bxCAN_IDtype
     * @{
     */
#define USR_CAN_IDTYPE_STANDARD 0x00  // 标准数据帧ID
#define USR_CAN_IDTYPE_EXTENDED 0x01  // 扩展数据帧ID
    /**
     * @}
     */

    /** @defgroup Adernal_bxCAN_Frametype
     * @{
     */
#define USR_CAN_FRAMETYPE_REMOTE 0x00  // 远程帧
#define USR_CAN_FRAMETYPE_DATA 0x00  // 数据帧
    /**
     * @}
     */

    /** @defgroup Adernal_bxCAN_TxID
     * @{
     */
#define __ADERNAL_CANID_INIT 0x002  // 初始化帧ID
#define __ADERNAL_CANID_CTRL 0x004  // 控制帧ID
    /**
     * @}
     */

    typedef struct {
        uint16_t ID; /*!< Specifies the identifier.
This parameter can be a value of:
@ref Adernal_bxCAN_TxID or @ref Adernal_bxCAN_RxID				     */

        uint8_t IDtype; /*!< Specifies the identifier type for the message that will be
transmitted.
This parameter can be a value of @ref Adernal_bxCAN_IDtype               */

        uint8_t FrameType; /*!< Specifies the frame type of the message that will be transmitted.
This parameter can be a value of @ref Adernal_bxCAN_Frametype            */

        uint8_t DataLength;      /*!< Specifies the length of the frame that will be transmitted.
This parameter must be a number between:1~8     */
    } Usr_CAN_TxHeader_Typedef;  // CAN发送帧结构体

    Usr_CAN_TxSuccess_Typedef Usr_CAN_TxMsg(Usr_CAN_TxHeader_Typedef* Usr_CAN_TxHeader,
                                            uint8_t* TxData) {
        /* USER Customize BEGIN CAN Tx implement */
        // 获取 CAN 实例（假设使用 can1）
        static bsp::CAN* can_instance = bsp::CAN::FindInstance(&hcan1);

        if (can_instance == nullptr) {
            return Adernal_Tx_Error;
        }

        int ret = -1;
        if (Usr_CAN_TxHeader->IDtype == USR_CAN_IDTYPE_STANDARD) {
            // 标准帧发送
            ret =
                can_instance->Transmit(Usr_CAN_TxHeader->ID, TxData, Usr_CAN_TxHeader->DataLength);
        } else if (Usr_CAN_TxHeader->IDtype == USR_CAN_IDTYPE_EXTENDED) {
            // 扩展帧发送
            ret = can_instance->TransmitExtend(Usr_CAN_TxHeader->ID, TxData,
                                               Usr_CAN_TxHeader->DataLength);
        }

        return (ret > 0) ? Adernal_Tx_OK : Adernal_Tx_Error;
        /* USER Customize END CAN Tx implement */
    }

    /**
     * @brief  Parse the information of the standby frame.Called in CAN receive interrupt callback.
     * @param  Usr_CAN_RxHeader_Typedef *Usr_CAN_RxHeader,The frame header of the CAN packet.
     * @param  uint8_t *RxData,The content of the CAN packet.
     * @retval Adernal_StdbyState_Typedef.Adernal_StdbyState_OK point to standingby is
     * ok,Adernal_StdbyState_Fail point to not standing by,Adernal_StdbyState_Error point to cannor
     * parse frame.
     */
    uint8_t Adernal_StdbyFrame_Unpack(Usr_CAN_RxHeader_Typedef* Usr_CAN_RxHeader, uint8_t* RxData) {
        /*Adernal_StdbyFrame_Unpack 是用于解析待机状态反馈帧的专用函数，主要用于处理 CAN
         * 接收中断中获取的待机状态数据。 返回超电控制器是否就绪*/
        // 判断帧头是否正确
        if (Usr_CAN_RxHeader->ID == __ADERNAL_CANID_STDBY &&
            Usr_CAN_RxHeader->IDtype == USR_CAN_IDTYPE_STANDARD &&
            Usr_CAN_RxHeader->FrameType == USR_CAN_FRAMETYPE_DATA &&
            Usr_CAN_RxHeader->DataLength == 1u) {
            uint8_t ReturnValue;
            // 解析RxData[0]的字节值
            switch (RxData[0]) {
                case 0x00: {
                    ReturnValue = Adernal_StdbyState_Fail;
                } break;
                case 0xFF: {
                    ReturnValue = Adernal_StdbyState_OK;
                } break;
                default: {
                    ReturnValue = Adernal_StdbyState_Error;
                }
            }

            return ReturnValue;
        } else {
            return Adernal_StdbyState_Error;
        }
    }

    /**
     * @brief  Send an initialization frame.
     * @param  Adernal_Init_Typedef Adernal_Init,Cap type select.
     * @retval Usr_CAN_TxSuccess_Typedef.
     */
    // 发送初始化帧，该帧每次上电后仅接收一次，用于设置超电控制器可充电的最高电压，无需回调函数
    Usr_CAN_TxSuccess_Typedef Adernal_Init(Adernal_Init_Typedef Adernal_Init) {
        /*Adernal_Init 函数是用于发送超级电容初始化控制帧的专用接口。*/
        // 创建 CAN 发送帧结构体
        Usr_CAN_TxHeader_Typedef Usr_CAN_TxHeader;
        Usr_CAN_TxHeader.ID = __ADERNAL_CANID_INIT;
        Usr_CAN_TxHeader.IDtype = USR_CAN_IDTYPE_STANDARD;
        Usr_CAN_TxHeader.FrameType = USR_CAN_FRAMETYPE_DATA;
        Usr_CAN_TxHeader.DataLength = 1u;
        // 将输入参数转换为 CAN 发送帧数据
        uint8_t TxData = (uint8_t)Adernal_Init;
        // 调用Usr_CAN_TxMsg发送构造好的CAN帧
        return Usr_CAN_TxMsg(&Usr_CAN_TxHeader, &TxData);
    }

    /**
     * @brief  Parse the information of the Feedback frame.Called in CAN receive interrupt callback.
     * @param  Usr_CAN_RxHeader_Typedef *Usr_CAN_RxHeader,The frame header of the CAN packet.
     * @param  uint8_t *RxData,The content of the CAN packet.
     * @param  Adernal_Fb_Typedef *Adernal_Fb,Stores and returns information that has been resolved.
     * @retval Adernal_FbState_Typedef.Adernal_FbState_OK point to parse is ok,Adernal_FbState_Error
     * point to cannor parse frame.
     */
    Adernal_FbState_Typedef Adernal_FbFrame_Unpack(Usr_CAN_RxHeader_Typedef* Usr_CAN_RxHeader,
                                                   uint8_t* RxData,
                                                   Adernal_Fb_Typedef* Adernal_Fb) {
        /*Adernal_FbFrame_Unpack
         * 是用于解析超级电容反馈数据帧的核心函数，主要处理电压和功率等关键参数的实时反馈*/
        // 判断帧头是否正确
        if (Usr_CAN_RxHeader->ID == __ADERNAL_CANID_FB &&
            Usr_CAN_RxHeader->IDtype == USR_CAN_IDTYPE_STANDARD &&
            Usr_CAN_RxHeader->FrameType == USR_CAN_FRAMETYPE_DATA &&
            Usr_CAN_RxHeader->DataLength == 6u) {
            // 电压解析
            uint16_t V_Handler = 0x0000;    // 电压组低16位
            V_Handler = (RxData[0] << 8u);  // 添加高8位的电压值到V_Handler中
            V_Handler += RxData[1];         // 将低8位的电压值添加到V_Handler中
            Adernal_Fb->Voltage_NoESR = 0.01f * ((float)V_Handler);  //  将电压值转换为实际电压
            // 功率解析
            uint16_t P_Handler = 0x0000;    // 功率组低16位
            P_Handler = (RxData[4] << 8u);  // 将高8位的功率值添加到P_Handler中
            P_Handler += RxData[5];         // 将低8位的功率值添加到P_Handler中
            Adernal_Fb->Power_Battery = 0.01f * ((float)P_Handler);  // 将功率值转换为实际功率
            // 工作模式解析
            Adernal_Fb->Work_Sentry1 = RxData[2];
            Adernal_Fb->Work_Sentry2 = RxData[3];
            // 返回结果
            return Adernal_FbState_OK;
        } else {
            return Adernal_FbState_Error;
        }
    }

    /**
     * @brief  Send an control frame.
     * @param  Adernal_Ctrl_Typedef *Adernal_Ctrl,Control result Setting.
     * @retval Usr_CAN_TxSuccess_Typedef.
     */
    Usr_CAN_TxSuccess_Typedef Adernal_Ctrl(Adernal_Ctrl_Typedef* Adernal_Ctrl) {
        /*Adernal_Ctrl 函数是用于发送超级电容控制指令帧的核心接口，通过 CAN
         * 总线向超级电容模块发送功率设定、保护阈值和工作模式等控制参数*/
        // 创建 CAN 发送帧结构体
        Usr_CAN_TxHeader_Typedef Usr_CAN_TxHeader;
        Usr_CAN_TxHeader.ID = __ADERNAL_CANID_CTRL;
        Usr_CAN_TxHeader.IDtype = USR_CAN_IDTYPE_STANDARD;
        Usr_CAN_TxHeader.FrameType = USR_CAN_FRAMETYPE_DATA;
        Usr_CAN_TxHeader.DataLength = 3u;
        // 数据封装
        uint8_t TxData[3];
        TxData[0] = Adernal_Ctrl->ExpectPwr;  // 设置期望功率
        TxData[1] = Adernal_Ctrl->Exceed;     // 超频阈值
        TxData[2] = Adernal_Ctrl->Mode;       // 工作模式
        // 调用Usr_CAN_TxMsg发送构造好的CAN帧
        return Usr_CAN_TxMsg(&Usr_CAN_TxHeader, TxData);
    }

    /**
     * @brief  Parse the information of the Safety frame.Called in CAN receive interrupt callback.
     * @param  Usr_CAN_RxHeader_Typedef *Usr_CAN_RxHeader,The frame header of the CAN packet.
     * @param  uint8_t *RxData,The content of the CAN packet.
     * @param  Adernal_SafetyLevel_Typedef *Adernal_SafetyLevel,Point to an 8-byte array to store
     * parse results.
     * @retval Adernal_SafetyState_Typedef.Adernal_SafetyState_OK point to parse is
     * ok,Adernal_SafetyState_Error point to cannor parse frame.
     */
    Adernal_SafetyState_Typedef Adernal_Safety_Unpack(
        Usr_CAN_RxHeader_Typedef* Usr_CAN_RxHeader, uint8_t* RxData,
        Adernal_SafetyLevel_Typedef* Adernal_SafetyLevel) {
        /*Adernal_Safety_Unpack
         * 是用于解析超级电容安全状态帧的关键函数，主要用于实时监测电容模块的故障和安全等级信息。*/
        // 判断帧头是否正确
        if (Usr_CAN_RxHeader->ID == __ADERNAL_CANID_SAFE &&
            Usr_CAN_RxHeader->IDtype == USR_CAN_IDTYPE_STANDARD &&
            Usr_CAN_RxHeader->FrameType == USR_CAN_FRAMETYPE_DATA &&
            Usr_CAN_RxHeader->DataLength == 8u) {
            // 将8字节数据按原始值存入安全等级数组
            uint8_t i;
            for (i = 0; i < 8; i++) {
                Adernal_SafetyLevel[i] = (Adernal_SafetyLevel_Typedef)RxData[i];
            }
            // 返回结果
            return Adernal_SafetyState_OK;
        } else {
            // 返回结果
            return Adernal_SafetyState_Error;
        }
    }

    // 初始化静态成员
    Adernal_SuperCap* Adernal_SuperCap::instance_ = nullptr;

    // 静态回调函数实现
    void Adernal_SuperCap::ReadyFrameCallback(const uint8_t data[], void* args) {
        UNUSED(args);
        if (instance_) {
            instance_->handleReadyFrame(data);
        }
    }

    void Adernal_SuperCap::FeedbackFrameCallback(const uint8_t data[], void* args) {
        UNUSED(args);
        if (instance_) {
            instance_->handleFeedbackFrame(data);
        }
    }

    void Adernal_SuperCap::SafetyFrameCallback(const uint8_t data[], void* args) {
        UNUSED(args);
        if (instance_) {
            instance_->handleSafetyFrame(data);
        }
    }

    Adernal_SuperCap::Adernal_SuperCap(bsp::CAN* can_instance) : can_(can_instance) {
        // 保存实例指针用于静态回调
        instance_ = this;

        // 注册CAN回调函数
        can_->RegisterRxCallback(__ADERNAL_CANID_STDBY, ReadyFrameCallback, nullptr);
        can_->RegisterRxCallback(__ADERNAL_CANID_FB, FeedbackFrameCallback, nullptr);
        can_->RegisterRxCallback(__ADERNAL_CANID_SAFE, SafetyFrameCallback, nullptr);

        // 初始化控制结构
        current_ctrl_.ExpectPwr = 50;  // 默认50W
        current_ctrl_.Mode = Adernal_CtrlMode_Work;
        current_ctrl_.Exceed = Adernal_CtrlExceed_Off;
    }

    Adernal_SuperCap::~Adernal_SuperCap() {
        // 清除实例指针
        if (instance_ == this) {
            instance_ = nullptr;
        }
    }

    bool Adernal_SuperCap::initialize(Adernal_Init_Typedef cap_type) {
        return Adernal_Init(cap_type) == Adernal_Tx_OK;
    }

    bool Adernal_SuperCap::setControl(uint8_t expect_power, Adernal_CtrlMode_Typedef mode,
                                      Adernal_CtrlExceed_Typedef exceed) {
        current_ctrl_.ExpectPwr = expect_power;
        current_ctrl_.Mode = mode;
        current_ctrl_.Exceed = exceed;

        return Adernal_Ctrl(&current_ctrl_) == Adernal_Tx_OK;
    }

    void Adernal_SuperCap::handleReadyFrame(const uint8_t data[]) {
        Usr_CAN_RxHeader_Typedef rx_header;
        rx_header.ID = __ADERNAL_CANID_STDBY;
        rx_header.IDtype = USR_CAN_IDTYPE_STANDARD;
        rx_header.FrameType = USR_CAN_FRAMETYPE_DATA;
        rx_header.DataLength = 1;

        uint8_t state = Adernal_StdbyFrame_Unpack(&rx_header, (uint8_t*)data);
        ready_ = (state == Adernal_StdbyState_OK);
        new_ready_ = true;
    }

    void Adernal_SuperCap::handleFeedbackFrame(const uint8_t data[]) {
        Usr_CAN_RxHeader_Typedef rx_header;
        rx_header.ID = __ADERNAL_CANID_FB;
        rx_header.IDtype = USR_CAN_IDTYPE_STANDARD;
        rx_header.FrameType = USR_CAN_FRAMETYPE_DATA;
        rx_header.DataLength = 6;

        if (Adernal_FbFrame_Unpack(&rx_header, (uint8_t*)data, &feedback_) == Adernal_FbState_OK) {
            new_feedback_ = true;
        }
    }

    void Adernal_SuperCap::handleSafetyFrame(const uint8_t data[]) {
        Usr_CAN_RxHeader_Typedef rx_header;
        rx_header.ID = __ADERNAL_CANID_SAFE;
        rx_header.IDtype = USR_CAN_IDTYPE_STANDARD;
        rx_header.FrameType = USR_CAN_FRAMETYPE_DATA;
        rx_header.DataLength = 8;

        if (Adernal_Safety_Unpack(&rx_header, (uint8_t*)data, safety_levels_) ==
            Adernal_SafetyState_OK) {
            new_safety_ = true;
        }
    }
}  // namespace driver