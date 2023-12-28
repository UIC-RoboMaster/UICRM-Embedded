/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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
#include "bsp_can.h"

namespace driver {
    /**
     * @brief 达妙4310电机的操作模式
     *
     * @note 电机的操作模式决定了电机的输出参数
     *
     * @enum MIT                  电机的输出参数为角度与扭矩，此模式仅限有经验的用户使用
     * @enum POS_VEL              电机的输出参数为位置和速度，电机的输出为位置和速度控制
     * @enum VEL                  电机的输出参数为速度，电机的输出为速度控制
     */
    /**
     * @brief operation mode of DM m4310 motor
     *
     * @note operation mode of motor determines the output parameters of motor
     *
     * @enum MIT                  output parameters are angle and torque, only for experienced users
     * @enum POS_VEL              output parameters are position and velocity, output is position
     * and velocity control
     * @enum VEL                  output parameters are velocity, output is velocity control
     */
    typedef enum {
        MIT = 0,
        POS_VEL = 1,
        VEL = 2,
    } dm_m4310_mode_t;

    /**
     * @brief 达妙4310电机的标准类
     *
     * @note 达妙电机使用CAN通信，但是达妙电机的CAN通信协议与DJI的不同，因此需要单独实现
     */
    /**
     * @brief DM m4310 motor class
     *
     * @note DM motor uses CAN communication, but the CAN protocol is different from DJI's, so it
     * needs to be implemented separately
     */
    class DMMotor4310 {
      public:
        /**
         * @brief 基础构造函数
         * @note 这个构造函数与MotorCANBase的构造函数不同。
         *      4310电机的CAN id与实际CAN id不同，因此需要单独指定
         *      不同模式下的CAN id如下：
         *      MIT:                  实际CAN id
         *      角度速度模式:    实际CAN id + 0x100
         *      纯速度模式:             实际CAN id + 0x200
         */
        /**
         *  @brief base constructor
         *  @note constructor wrapper is different from MotorCANBase
         *  CAN frame id for different modes:
         *      MIT:                  actual CAN id.
         *      position-velocity:    CAN id + 0x100.
         *      velocity:             CAN id + 0x200.
         *  @param can    CAN object
         *  @param rx_id  Master id
         *  @param tx_id  CAN id *** NOT the actual CAN id but the id configured in software ***
         *  @param mode   0: MIT
         *                1: position-velocity
         *                2: velocity
         */
        DMMotor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_m4310_mode_t mode);

        /**
         * @brief 更新电机的反馈数据
         *
         * @note 仅在CAN回调函数中使用，不要在其他地方调用
         *
         * @param data 原始数据
         */
        /**
         * @brief update the current theta for the motor
         *
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data raw data bytes
         */
        void UpdateData(const uint8_t data[]);

        /**
         * @brief 使能4310
         */
        /**
         * @brief enable 4310
         */
        void MotorEnable();
        /**
         * @brief 失能4310
         */
        /**
         * @brief disable 4310
         */
        void MotorDisable();
        /**
         * @brief 更新电机的零点
         */
        /**
         * @brief update zero position of motor
         */
        void SetZeroPos();

        /**
         * @brief 获取电机的角度，单位为[rad]
         *
         * @return 电机的弧度角度
         */
        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle
         */
        float GetTheta() const;

        /**
         * @brief 获取电机的角速度，单位为[rad / s]
         *
         * @return 电机的弧度角速度
         */
        /**
         * @brief get motor angular velocity, in [rad / s]
         * @return radian angular velocity
         */
        float GetOmega() const;

        /**
         * @brief 获取电机的扭矩，单位为[Nm]
         *
         * @return 电机的扭矩
         */
        /**
         * @brief get motor torque, in [Nm]
         * @return motor torque
         */
        float GetTorque() const;

        /**
         * @brief 直接传输数据到电机
         *
         * @note 由于一个4310电机需要使用一个独立的CAN id，因此这个函数不是静态的，直接由对象调用
         */
        /**
         * @brief transmit data to motor
         *
         * @note since a 4310 motor needs to use a separate CAN id, this function is not static,
         * directly called by the object
         */
        void TransmitOutput();

        /**
         * @brief 打印电机数据
         */
        /**
         * @brief print out motor data
         */
        void PrintData();

        /**
         * @brief 设置电机的输出参数
         * @note 本调用方式推荐用于MIT模式下，其他模式下请使用其他重载函数
         * @param position 电机的角度，单位为[rad]
         * @param velocity 电机的角速度，单位为[rad / s]
         * @param kp 电机的kp值，仅在MIT模式下有效
         * @param kd 电机的kd值，仅在MIT模式下有效
         * @param torque 电机的扭矩，仅在MIT模式下有效
         */
        /**
         * @brief set output parameters for m4310
         * @note this function is recommended for MIT mode, use other overloaded functions for other
         * modes
         * @param position motor angle, in [rad]
         * @param velocity motor angular velocity, in [rad / s]
         * @param kp kp value of motor, only valid in MIT mode
         * @param kd kd value of motor, only valid in MIT mode
         * @param torque motor torque, only valid in MIT mode
         */
        void SetOutput(float position, float velocity, float kp, float kd, float torque);

        /**
         * @brief 设置电机的输出参数
         * @note 本调用方式推荐用于角度速度模式下，其他模式下请使用其他重载函数
         * @param position 电机的角度，单位为[rad]
         * @param velocity 电机的角速度，单位为[rad / s]
         */
        /**
         * @brief set output parameters for m4310 using position-velocity mode
         * @note this function is recommended for position-velocity mode, use other overloaded
         * functions for other modes
         * @param position motor angle, in [rad]
         * @param velocity motor angular velocity, in [rad / s]
         */
        void SetOutput(float position, float velocity);

        /**
         * @brief 设置电机的输出参数
         * @note 本调用方式推荐用于纯速度模式下，其他模式下请使用其他重载函数
         * @param velocity 电机的角速度，单位为[rad / s]
         */
        /**
         * @brief set output parameters for m4310 using velocity mode
         * @note this function is recommended for velocity mode, use other overloaded functions for
         * other modes
         * @param velocity motor angular velocity, in [rad / s]
         */
        void SetOutput(float velocity);

        /**
         * @brief 转换函数，将float转换为unsigned int，给定范围和位数；
         *     详见m4310 V1.2文档
         *
         *     @param x 要转换的值
         *     @param x_min 当前参数的最小值
         *     @param x_max 当前参数的最大值
         *     @param bits 位数
         *     @return 从float转换为unsigned int的值
         */
        /**
         * @brief Converts a float to an unsigned int, given range and number of bits;
         *      see m4310 V1.2 document for detail
         * @param x value to be converted
         * @param x_min minimum value of the current parameter
         * @param x_max maximum value of the current parameter
         * @param bits size in bits
         * @return value converted from float to unsigned int
         */
        static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
        /**
         * @brief 转换函数，将unsigned int转换为float，给定范围和位数；
         *    详见m4310 V1.2文档
         * @param x_int 要转换的值
         * @param x_min 当前参数的最小值
         * @param x_max 当前参数的最大值
         * @param bits 位数
         * @return 从unsigned int转换为float的值
         */
        /**
         * @brief Converts an unsigned int to a float, given range and number of bits;
         *      see m4310 V1.2 document for detail
         * @param x_int value to be converted
         * @param x_min minimum value of the current parameter
         * @param x_max maximum value of the current parameter
         * @param bits size in bits
         * @return value converted from unsigned int to float
         */
        static float uint_to_float(int x_int, float x_min, float x_max, int bits);

        volatile bool connection_flag_ = false;

      private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
        uint16_t tx_id_actual_;

        volatile dm_m4310_mode_t mode_;     // current motor mode
        volatile float kp_set_ = 0;      // defined kp value
        volatile float kd_set_ = 0;      // defined kd value
        volatile float vel_set_ = 0;     // defined velocity
        volatile float pos_set_ = 0;     // defined position
        volatile float torque_set_ = 0;  // defined torque

        volatile int16_t raw_pos_ = 0;        // actual position
        volatile int16_t raw_vel_ = 0;        // actual velocity
        volatile int16_t raw_torque_ = 0;     // actual torque
        volatile int16_t raw_motorTemp_ = 0;  // motor temp
        volatile int16_t raw_mosTemp_ = 0;    // MOS temp
        volatile float theta_ = 0;            // actual angular position
        volatile float omega_ = 0;            // actual angular velocity
        volatile float torque_ = 0;           // actual torque

        constexpr static float KP_MIN = 0;
        constexpr static float KP_MAX = 500;
        constexpr static float KD_MIN = 0;
        constexpr static float KD_MAX = 5;
        constexpr static float P_MIN = -12.5;
        constexpr static float P_MAX = 12.5;
        constexpr static float V_MIN = -45;
        constexpr static float V_MAX = 45;
        constexpr static float T_MIN = -18;
        constexpr static float T_MAX = 18;
    };
}
