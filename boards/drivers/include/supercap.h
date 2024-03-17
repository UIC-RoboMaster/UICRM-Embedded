/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_can.h"
#include "connection_driver.h"

namespace driver {
    /**
     * @brief 超级电容初始化参数
     */
    /**
     * @brief SuperCap initialization parameters
     */
    typedef struct {
        bsp::CAN* can;
        uint16_t tx_id;
        uint16_t tx_settings_id;
        uint16_t rx_id;
    } supercap_init_t;

    /**
     * @brief 超级电容状态
     */
    /**
     * @brief SuperCap status
     */
    typedef union {
        uint8_t data;
        struct {
            uint8_t enable : 1;
            uint8_t cap_low_voltage : 1;
            uint8_t power_overload : 1;
            uint8_t : 3;
            uint8_t can_error : 1;
            uint8_t cap_error : 1;
        } __packed flags;
    } supercap_status_t;

    /**
     * @brief 超级电容发送标志位
     */
    /**
     * @brief SuperCap send flags
     */
    typedef union {
        uint8_t data;
        struct {
            uint8_t enable : 1;
            uint8_t : 7;
        } __packed flags;
    } supercap_send_flags_t;

    /**
     * @brief 超级电容管理
     * @details 通过CAN总线与超级电容通信，设置超级电容的参数，获取超级电容的状态
     * @note 本实例适配的超级电容型号为团队自研的超级电容，其他型号的超级电容可能无法使用
     */
    /**
     * @brief SuperCap management
     * @details Communicate with the supercapacitor through the CAN bus, set the parameters of the
     * supercapacitor, and obtain the status of the supercapacitor
     * @note This instance is suitable for the supercapacitor model independently developed by the
     * team, and other models of supercapacitors may not be available
     */
    class SuperCap : public ConnectionDriver {
      public:
        /**
         * @brief 构造函数
         * @param init 初始化结构体
         */
        /**
         * @brief Construct a new Super Cap object
         * @param init Initialization structure
         */
        SuperCap(supercap_init_t init);

        /**
         * @brief 使能超级电容
         * @note 本函数并不会立即使能超级电容，而是在下一次调用TransmitSettings()时才会生效
         */
        /**
         * @brief Enable supercapacitor
         * @note This function does not enable the supercapacitor immediately, but will take effect
         * the next time TransmitSettings() is called
         */
        void Enable();

        /**
         * @brief 禁用超级电容
         * @note 本函数并不会立即禁用超级电容，而是在下一次调用TransmitSettings()时才会生效
         */
        /**
         * @brief Disable supercapacitor
         * @note This function does not disable the supercapacitor immediately, but will take effect
         * the next time TransmitSettings() is called
         */
        void Disable();

        /**
         * @brief 设置超级电容所在电路的功率限制
         * @param power 总功率，单位为 [W]
         * @note 本函数并不会立即设置功率，而是在下一次调用TransmitSettings()时才会生效
         */
        /**
         * @brief Set the power limit of the circuit where the supercapacitor is located
         * @param power Total power, in [W]
         * @note This function does not set the power limit immediately, but will take effect the
         * next time TransmitSettings() is called
         */
        void SetPowerTotal(float power);

        /**
         * @brief 设置超级电容的最大充电功率
         * @param power 最大充电功率，单位为 [W]
         * @note 本函数并不会立即设置功率，而是在下一次调用TransmitSettings()时才会生效
         */
        /**
         * @brief Set the maximum charging power of the supercapacitor
         * @param power Maximum charging power, in [W]
         * @note This function does not set the power immediately, but will take effect the next
         * time TransmitSettings() is called
         */
        void SetMaxChargePower(float power);

        /**
         * @brief 设置超级电容的最大放电功率
         * @param power 最大放电功率，单位为 [W]
         * @note 本函数并不会立即设置功率，而是在下一次调用TransmitSettings()时才会生效
         */
        /**
         * @brief Set the maximum discharge power of the supercapacitor
         * @param power Maximum discharge power, in [W]
         * @note This function does not set the power immediately, but will take effect the next
         * time TransmitSettings() is called
         */
        void SetMaxDischargePower(float power);

        /**
         * @brief 设置超级电容的偏好缓冲电量，非RM比赛请设置为0
         * @param buffer 偏好缓冲电量，单位为 [J]
         * @note 本函数并不会立即设置缓冲电量，而是在下一次调用TransmitSettings()时才会生效
         */
        /**
         * @brief Set the preferred buffer power of the supercapacitor, please set to 0 for
         * non-RoboMaster competitions
         * @param buffer Preferred buffer power, in [J]
         * @note This function does not set the buffer power immediately, but will take effect the
         * next time TransmitSettings() is called
         */
        void SetPerferBuffer(float buffer);

        /**
         * @brief 传输超级电容的设置
         */
        /**
         * @brief Transmit supercapacitor settings
         */
        void TransmitSettings();

        /**
         * @brief 更新实时功率缓冲能量
         * @param buffer 实时缓冲能量，单位为 [J]
         * @note 这个函数会直接更新超级电容的缓冲能量，不需要调用TransmitSettings()函数
         */
        /**
         * @brief Update real-time power buffer energy
         * @param buffer Real-time buffer energy, in [J]
         * @note This function will directly update the buffer energy of the supercapacitor without
         * calling the TransmitSettings() function
         */
        void UpdateCurrentBuffer(float buffer);

        /**
         * @brief 获取超级电容的电压
         * @return float 超级电容的电压，单位为 [V]
         */
        /**
         * @brief Get the voltage of the supercapacitor
         * @return float Voltage of the supercapacitor, in [V]
         */
        float GetCapVoltage() const;

        /**
         * @brief 获取超级电容的功率
         * @note 充电为正数，放电为负数
         * @return 超级电容的功率，单位为 [W]
         */
        /**
         * @brief Get the power of the supercapacitor
         * @note Charging is positive, discharging is negative
         * @return Power of the supercapacitor, in [W]
         */
        float GetCapPower() const;

        /**
         * @brief 获取当前链路总输出功率
         * @return 当前链路总输出功率，单位为 [W]
         */
        /**
         * @brief Get the total output power of the current link
         * @return Total output power of the current link, in [W]
         */
        float GetOutputPower() const;

        /**
         * @brief 获取当前链路总输出电压
         * @return 当前链路总输出电压，单位为 [V]
         */
        /**
         * @brief Get the total output voltage of the current link
         * @return Total output voltage of the current link, in [V]
         */
        float GetOutputVoltage() const;

        /**
         * @brief 获得超级电容的状态
         * @return 超级电容的状态
         */
        /**
         * @brief Get the status of the supercapacitor
         * @return Status of the supercapacitor
         */
        supercap_status_t GetStatus() const;

        void SetMaxVoltage(float voltage);

        float GetPercentage() const;

        /**
         * @brief 更新超级电容的数据与状态
         * @note 本函数会在CAN总线接收到数据时被调用，不应该在其他地方调用
         * @param data CAN总线接收到的数据
         */
        /**
         * @brief Update the data and status of the supercapacitor
         * @note This function will be called when data is received on the CAN bus and should not be
         * called elsewhere
         * @param data Data received on the CAN bus
         */
        void UpdateData(const uint8_t* data);

        /**
         * @brief can回调函数
         * @note 本函数会在CAN总线接收到数据时被调用，不应该在其他地方调用
         * @param data CAN总线接收到的数据
         * @param args 本类的指针
         */
        /**
         * @brief can callback function
         * @note This function will be called when data is received on the CAN bus and should not be
         * called elsewhere
         * @param data Data received on the CAN bus
         * @param args Pointer to this class
         */
        static void CallbackWrapper(const uint8_t* data, void* args);

      private:
        bsp::CAN* can_;
        uint16_t tx_id_;
        uint16_t tx_settings_id_;
        uint16_t rx_id_;

        float cap_voltage_max_ = 26.0f;
        volatile float cap_voltage_ = 0.0f;
        volatile float cap_power_ = 0.0f;
        volatile float output_power_ = 0.0f;
        volatile float output_voltage_ = 0.0f;
        supercap_status_t status_ = {0};

        float power_total_ = 30.0f;
        float max_charge_power_ = 150.0f;
        float max_discharge_power_ = 250.0f;
        volatile float perfer_buffer_ = 45.0f;
        supercap_send_flags_t tx_flags_ = {0};

    };
}  // namespace driver
