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

#ifndef __ADERNAL_SDK
#define __ADERNAL_SDK

#include "bsp_can.h"
#include "main.h"
namespace driver {
#define USR_CAN_IDTYPE_STANDARD 0x00  // 标准数据帧ID
#define USR_CAN_FRAMETYPE_DATA 0x00   // 数据帧
                                      /** @defgroup Adernal_bxCAN_RxID
                                       * @{
                                       */
#define __ADERNAL_CANID_STDBY 0x005   // 就绪帧
#define __ADERNAL_CANID_FB 0x003      // 反馈帧
#define __ADERNAL_CANID_SAFE 0x001    // 安全提示帧
    /**
     * @}
     */

    /**
     * @brief  Adernal bxCAN Tx structure and enum definition
     */
    typedef struct {
        uint16_t ID; /*!< Specifies the identifier.
This parameter can be a value of:
@ref Adernal_bxCAN_TxID or @ref Adernal_bxCAN_RxID
ID可以为tx的ID或者rx的ID*/

        uint8_t IDtype; /*!< Specifies the identifier type for the message
 received.
 This parameter can be a value of @ref Adernal_bxCAN_IDtype
 ID类型*/

        uint8_t FrameType; /*!< Specifies the frame type of the message received.
            This parameter can be a value of @ref Adernal_bxCAN_Frametype
            帧类型*/

        uint8_t DataLength;      /*!< Specifies the length of the frame received.
                   This parameter must be a number between:1~8
                   数据长度*/
    } Usr_CAN_RxHeader_Typedef;  // CAN接收端结构体

    typedef enum { Adernal_Tx_OK, Adernal_Tx_Error } Usr_CAN_TxSuccess_Typedef;  // CAN是否发送成功

    /**
     * @brief  Adernal Standing By Frame structure and enum definition
     */
    typedef enum {
        Adernal_StdbyState_Error,
        Adernal_StdbyState_OK,
        Adernal_StdbyState_Fail
    } Adernal_StdbyState_Typedef;  // 就绪帧状态

    /**
     * @brief  Adernal initialization Frame structure and enum definition
     */
    typedef enum {
        Adernal_Init_CapType1,  // MAX:24V
        Adernal_Init_CapType2,  // MAX:28V
        Adernal_Init_CapType3   // MAX:30V
    } Adernal_Init_Typedef;     // 初始化帧中对超级电容类型的选择

    /**
     * @brief  Adernal Feedback Frame structure and enum definition
     */
    typedef struct {
        float Voltage_NoESR;  // 未ESR修正后的电容组电压
                              // /*!< ESR-corrected voltage value */

        float Power_Battery;  // 电源输入功率						/*!< Bus
                              // power */

        uint8_t Work_Sentry1;  // 工作强度1							/*!<
                               // Work sentry 1,point to curent limit ability */

        uint8_t Work_Sentry2;  // 工作强度2							/*!<
                               // Work sentry 2,point to power limit ability */
    } Adernal_Fb_Typedef;      // 反馈帧结构体

    typedef enum {
        Adernal_FbState_OK,
        Adernal_FbState_Error
    } Adernal_FbState_Typedef;  // 反馈帧状态

    /**
     * @brief  Adernal Control Frame structure and enum definition
     */
    typedef struct {
        uint8_t ExpectPwr; /*!< Specifies the Expect bus Power.
     This parameter must be a number between:35~210
     期望总功率 35~210W*/

        uint8_t Exceed; /*!< Specifies the Exceed on or off.
This parameter must be a value of enum Adernal_CtrlExceed_Typedef
超频开关*/

        uint8_t Mode;        /*!< Specifies the Selected Mode
This parameter must be a value of enum Adernal_CtrlMode_Typedef
模式选择*/
    } Adernal_Ctrl_Typedef;  // 控制帧结构体

    typedef enum {
        Adernal_CtrlExceed_Off,
        Adernal_CtrlExceed_On
    } Adernal_CtrlExceed_Typedef;  // 是否开启超频

    typedef enum {
        Adernal_CtrlMode_Silent,
        Adernal_CtrlMode_Work,
        Adernal_CtrlMode_Charge
    } Adernal_CtrlMode_Typedef;  // 模式选择

    /**
     * @brief  Adernal Safety Frame structure and enum definition
     */
    typedef enum {
        Adernal_SafetyState_OK,
        Adernal_SafetyState_Error
    } Adernal_SafetyState_Typedef;  // 安全提示帧状态

    typedef enum {
        Adernal_SafeLevel_Safe,
        Adernal_SafeLevel_Warning,
        Adernal_SafeLevel_Risk,
        Adernal_SafeLevel_Danger,
        Adernal_SafeLevel_Deadliness
    } Adernal_SafetyLevel_Typedef;  // 安全提示等级

    uint8_t Adernal_StdbyFrame_Unpack(Usr_CAN_RxHeader_Typedef* Usr_CAN_RxHeader, uint8_t* RxData);
    // 解包就绪帧
    Usr_CAN_TxSuccess_Typedef Adernal_Init(Adernal_Init_Typedef Adernal_Init);
    // 初始化
    Adernal_FbState_Typedef Adernal_FbFrame_Unpack(Usr_CAN_RxHeader_Typedef* Usr_CAN_RxHeader,
                                                   uint8_t* RxData, Adernal_Fb_Typedef* Adernal_Fb);
    // 解包反馈帧
    Usr_CAN_TxSuccess_Typedef Adernal_Ctrl(Adernal_Ctrl_Typedef* Adernal_Ctrl);
    // 控制
    Adernal_SafetyState_Typedef Adernal_Safety_Unpack(
        Usr_CAN_RxHeader_Typedef* Usr_CAN_RxHeader, uint8_t* RxData,
        Adernal_SafetyLevel_Typedef* Adernal_SafetyLevel);
    // 解包安全提示帧

    class Adernal_SuperCap {
      public:
        // 构造函数
        Adernal_SuperCap(bsp::CAN* can_instance);

        // 析构函数
        ~Adernal_SuperCap();

        // 初始化方法
        bool initialize(Adernal_Init_Typedef cap_type = Adernal_Init_CapType1);

        // 设置控制参数
        bool setControl(uint8_t expect_power,
                        Adernal_CtrlMode_Typedef mode = Adernal_CtrlMode_Silent,
                        Adernal_CtrlExceed_Typedef exceed = Adernal_CtrlExceed_Off);

        // 获取状态信息
        bool isReady() const {
            return ready_;
        }
        const Adernal_Fb_Typedef& getFeedback() const {
            return feedback_;
        }
        const Adernal_SafetyLevel_Typedef* getSafetyLevels() const {
            return safety_levels_;
        }

        // 状态更新标志
        bool hasNewFeedback() const {
            return new_feedback_;
        }
        bool hasNewSafetyInfo() const {
            return new_safety_;
        }
        bool hasNewReadyStatus() const {
            return new_ready_;
        }

        // 清除状态更新标志
        void clearFeedbackFlag() {
            new_feedback_ = false;
        }
        void clearSafetyFlag() {
            new_safety_ = false;
        }
        void clearReadyFlag() {
            new_ready_ = false;
        }

        // 处理接收到的CAN消息的回调函数 - 公开以便静态回调函数访问
        void handleReadyFrame(const uint8_t data[]);
        void handleFeedbackFrame(const uint8_t data[]);
        void handleSafetyFrame(const uint8_t data[]);

      private:
        bsp::CAN* can_;                                 // CAN实例
        bool ready_ = false;                            // 就绪状态
        Adernal_Fb_Typedef feedback_;                   // 反馈数据
        Adernal_SafetyLevel_Typedef safety_levels_[8];  // 安全等级

        // 状态更新标志
        bool new_ready_ = false;
        bool new_feedback_ = false;
        bool new_safety_ = false;

        // 当前控制配置
        Adernal_Ctrl_Typedef current_ctrl_;

        // 静态回调函数
        static void ReadyFrameCallback(const uint8_t data[], void* args);
        static void FeedbackFrameCallback(const uint8_t data[], void* args);
        static void SafetyFrameCallback(const uint8_t data[], void* args);

        // 静态实例指针，用于回调函数中
        static Adernal_SuperCap* instance_;
    };
}  // namespace driver
#endif