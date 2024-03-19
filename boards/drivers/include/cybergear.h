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

#include "bsp_can.h"
#include "connection_driver.h"
#include "main.h"

namespace driver {
    /*
     * @brief 电机模式
     */
    /**
     * @brief Motor mode
     */
    typedef enum {
        CYBERGEAR_MODE_CURRENT = 3,  /**< 电流模式 */
        CYBERGEAR_MODE_SPEED = 2,    /**< 速度模式 */
        CYBERGEAR_MODE_POSITION = 1, /**< 位置模式 */
        CYBERGEAR_MODE_MIT = 0,      /**< MIT模式 */
    } cybergear_mode_e;
    /**
     * @brief 电机CAN扩展ID的数据结构
     */
    /**
     * @brief Data structure of motor CAN extended ID
     */
    typedef struct {
        uint32_t id : 8;    /**< 电机ID */
        uint32_t data : 16; /**< 数据 */
        uint32_t mode : 5;  /**< 数据类型 */
        uint32_t res : 3;   /**< 保留 */
    } cybergear_extid_t;

    /**
     * @brief 电机错误类型
     */
    /**
     * @brief Motor error type
     */
    typedef enum {
        CYBERGEAR_ERROR_NONE = 0,             /**< 无错误 */
        CYBERGEAR_ERROR_BATTERY_LOW = 1,      /**< 电池电压过低 */
        CYBERGEAR_ERROR_OVER_CURRENT = 2,     /**< 过流 */
        CYBERGEAR_ERROR_OVER_TEMPERATURE = 3, /**< 过温 */
        CYBERGEAR_ERROR_MAGNETIC = 4,         /**< 磁编码器错误 */
        CYBERGEAR_ERROR_ENCODER = 5,          /**< 光电编码器错误 */
        CYBERGEAR_ERROR_NO_CAILBRATION = 6,   /**< 未校准 */
    } cybergear_error_e;
    /**
     * @brief 电机状态
     */
    /**
     * @brief Motor status
     */
    typedef enum {
        CYBERGEAR_STATUS_RESET = 0,  /**< 重置 */
        CYBERGEAR_STATUS_CALI = 1,   /**< 校准 */
        CYBERGEAR_STATUS_NORMAL = 2, /**< 正常 */
    } cybergear_status_e;
    /**
     * @brief 小米CyberGear电机类
     */
    /**
     * @brief Xiaomi CyberGear motor class
     */
    class CyberGear : public driver::ConnectionDriver {
      public:
        /**
         * @brief Construct a new CyberGear object
         * @param can CAN总线指针
         * @param tx_id 主机ID
         * @param rx_id 电机ID
         */
        /**
         * @brief Construct a new CyberGear object
         * @param can CAN bus pointer
         * @param tx_id Master ID
         * @param rx_id Motor ID
         */
        CyberGear(bsp::CAN* can, uint8_t tx_id, uint8_t rx_id);
        /**
         * @brief Destroy the CyberGear object
         */
        ~CyberGear();

        /**
         * @brief 更新电机数据
         * @param data can数据
         * @param ext_id can的扩展ID
         * @note 仅在CAN回调函数中使用，不要在其他地方调用
         */
        /**
         * @brief Update motor data
         * @param data can data
         * @param ext_id can extended ID
         * @note Only use in CAN callback function, do not call elsewhere
         */
        void UpdateData(const uint8_t data[], const uint32_t ext_id);

        /**
         * @brief 使能电机
         * @note 电机上电后默认为失能状态
         */
        /**
         * @brief Enable motor
         * @note The motor is disabled by default after power-on
         */
        void MotorEnable();
        /**
         * @brief 失能电机
         */
        /**
         * @brief Disable motor
         */
        void MotorDisable();

        /**
         * @brief 重置电机
         */
        /**
         * @brief Reset motor
         */
        void Reset();

        // void ReadIndex(uint16_t index);

        /**
         * @brief 写入电机参数
         * @param index 参数索引
         * @param data 参数值
         * @note 参考电机手册
         */
        /**
         * @brief Write motor parameters
         * @param index Parameter index
         * @param data Parameter value
         * @note Refer to the motor manual
         */
        void WriteIndex(uint16_t index, uint8_t data);
        /**
         * @brief 写入电机参数
         * @param index 参数索引
         * @param data 参数值
         * @note 参考电机手册
         */
        /**
         * @brief Write motor parameters
         * @param index Parameter index
         * @param data Parameter value
         * @note Refer to the motor manual
         */
        void WriteIndex(uint16_t index, uint16_t data);
        /**
         * @brief 写入电机参数
         * @param index 参数索引
         * @param data 参数值
         * @note 参考电机手册
         */
        /**
         * @brief Write motor parameters
         * @param index Parameter index
         * @param data Parameter value
         * @note Refer to the motor manual
         */
        void WriteIndex(uint16_t index, float data);
        /**
         * @brief 设置电机运行模式
         * @param mode 电机运行模式
         */
        /**
         * @brief Set motor operation mode
         * @param mode Motor operation mode
         */
        void SetMode(cybergear_mode_e mode);

        /**
         * @brief 设置电机电流，如果是速度模式则为速度
         * @param output 电机电流或速度
         */
        /**
         * @brief Set motor current, if it is speed mode, it is speed
         * @param output Motor current or speed
         */
        void SetOutput(float output);

        /**
         * @brief 设置电机的角度和角速度
         * @param position 电机角度
         * @param velocity 电机角速度
         */
        /**
         *  @brief Set the angle and angular velocity of the motor
         * @param position Motor angle
         * @param velocity Motor angular velocity
         */
        void SetOutout(float position, float velocity);

        /**
         * @brief 设置电机的角度、角速度和力矩(MIT模式)
         * @param position Motor angle
         * @param velocity Motor angular velocity
         * @param torque Motor torque
         * @param kp MIT模式的kp
         * @param kd MIT模式的kd
         */
        /**
         * @brief Set the angle, angular velocity and torque of the motor (MIT mode)
         * @param position Motor angle
         * @param velocity Motor angular velocity
         * @param torque Motor torque
         * @param kp kp of MIT mode
         * @param kd kd of MIT mode
         */
        void SetOutput(float position, float velocity, float torque, float kp, float kd);

        /**
         * @brief 获得电机的角度，格式为[rad]
         *
         * @return 电机的弧度角度，范围为[0, 2PI]
         */
        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle, range between [0, 2PI]
         */
        float GetTheta() const;

        /**
         * @brief 获得电机的角度与目标角度的角度差，格式为[rad]
         *
         * @param target 目标角度，格式为[rad]
         *
         * @return 与目标角度的弧度角度差，范围为[-PI, PI]
         */
        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        float GetThetaDelta(const float target) const;

        /**
         * @brief 获得电机的角度，格式为[rad]
         *
         * @return 电机的弧度角度，MIT模式下为[-4PI, 4PI]
         */
        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle, range between [-4PI, 4PI] in MIT mode
         */
        float GetMITTheta() const;

        /**
         * @brief 获得电机的角度与目标角度的角度差，格式为[rad]
         * @param target 目标角度，格式为[rad]
         * @return 与目标角度的弧度角度差
         */
        float GetMITThetaDelta(const float target) const;

        /**
         * @brief 获得电机的角速度，格式为[rad / s]
         *
         * @return 电机的角速度
         */
        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @return angular velocity
         */
        float GetOmega() const;

        /**
         * @brief 获得电机的角速度与目标角速度的角速度差，格式为[rad / s]
         *
         * @param target 目标角速度，格式为[rad / s]
         *
         * @return 与目标角速度的角速度差
         */
        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        float GetOmegaDelta(const float target) const;

        /**
         * @brief 获得电机的扭矩，格式为[Nm]
         *
         * @return 电机的扭矩
         */
        /**
         * @brief get motor torque, in [Nm]
         * @return motor torque
         */
        float GetTorque() const;

        /**
         * @brief 获得电机的温度，格式为[℃]
         * @return 电机的温度
         */
        float GetTemp() const;

        void PrintData();

      private:
        bsp::CAN* can_;
        uint8_t tx_id_;
        uint8_t rx_id_;
        cybergear_extid_t tx_ext_id_ = {
            .id = 0,
            .data = 0,
            .mode = 0,
            .res = 0,
        };
        cybergear_extid_t rx_ext_id_ = {
            .id = 0,
            .data = 0,
            .mode = 0,
            .res = 0,
        };
        uint8_t tx_data_[8] = {0};

        cybergear_mode_e mode_ = CYBERGEAR_MODE_CURRENT;

        float theta_ = 0;
        float mit_theta_ = 0;
        float omega_ = 0;
        float torque_ = 0;
        float temperature_ = 0;
        float current_ = 0;

        uint8_t error_code_ = 0;
        uint8_t status_ = 0;

        void TransmitData();

        static int16_t float_to_uint(float x, float x_min, float x_max, int bits);

        static float uint_to_float(int x, float x_min, float x_max, int bits);
    };

};  // namespace driver