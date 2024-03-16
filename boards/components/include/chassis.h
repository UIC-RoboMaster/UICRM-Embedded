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

#pragma once

#include "MotorCanBase.h"
#include "can_bridge.h"
#include "pid.h"
#include "power_limit.h"
#include "supercap.h"

#define MAX_WHEEL_NUM 8

namespace control {

    /**
     * @brief chassis models
     */
    typedef enum { CHASSIS_MECANUM_WHEEL,CHASSIS_OMNI_WHEEL } chassis_model_t;

    /**
     * @brief structure used when chassis instance is initialized
     */
    struct chassis_t {
        driver::MotorCANBase** motors; /* motor instances of all chassis motors */
        chassis_model_t model;         /* chassis model                         */
        float offset = 0;
        bool power_limit_on = false;
        bool has_super_capacitor = false;
        driver::SuperCap* super_capacitor = nullptr;
    };

    /**
     * @brief motor configs for four wheel vehicles
     */
    struct FourWheel {
        enum { front_left, front_right, back_left, back_right, motor_num };
    };

    /**
     * @brief wrapper class for chassis
     */
    class Chassis {
      public:
        /**
         * @brief constructor for chassis
         *
         * @param chassis structure that used to initialize chassis, refer to type
         * chassis_t
         */
        Chassis(const chassis_t chassis);

        /**
         * @brief destructor for chassis
         */
        ~Chassis();

        /**
         * @brief set the speed for chassis motors
         *
         * @param x_speed chassis speed on x-direction
         * @param y_speed chassis speed on y-direction
         * @param turn_speed chassis clockwise turning speed
         */
        void SetSpeed(const float x_speed, const float y_speed = 0, const float turn_speed = 0);

        /**
         * @brief set the power limit for chassis motors
         * @param power_limit_on whether to enable power limit
         * @param power_limit total power limit, in [W]
         * @param chassis_power Current chassis power, in [W]
         * @param chassis_power_buffer Current chassis power buffer, in [J]
         */
        void SetPower(bool power_limit_on, float power_limit, float chassis_power,
                      float chassis_power_buffer, bool enable_supercap = false);

        void Enable();

        void Disable();

        /**
         * @brief calculate the output of the motors under current configuration
         * @note does not command the motor immediately
         */
        void Update();

        void CanBridgeSetTxId(uint8_t tx_id);

        static void CanBridgeUpdateEventXYWrapper(communication::can_bridge_ext_id_t ext_id,
                                                  communication::can_bridge_data_t data,
                                                  void* args);

        static void CanBridgeUpdateEventTurnWrapper(communication::can_bridge_ext_id_t ext_id,
                                                    communication::can_bridge_data_t data,
                                                    void* args);

        static void CanBridgeUpdateEventPowerLimitWrapper(communication::can_bridge_ext_id_t ext_id,
                                                          communication::can_bridge_data_t data,
                                                          void* args);

        static void CanBridgeUpdateEventCurrentPowerWrapper(
            communication::can_bridge_ext_id_t ext_id, communication::can_bridge_data_t data,
            void* args);

        void CanBridgeUpdateEventXY(communication::can_bridge_ext_id_t ext_id,
                                    communication::can_bridge_data_t data);

        void CanBridgeUpdateEventTurn(communication::can_bridge_ext_id_t ext_id,
                                      communication::can_bridge_data_t data);

        void CanBridgeUpdateEventPowerLimit(communication::can_bridge_ext_id_t ext_id,
                                            communication::can_bridge_data_t data);

        void CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                              communication::can_bridge_data_t data);

        static void UpdatePowerLimitWrapper(void* args);

        void UpdatePowerLimit();

        void SetMaxMotorSpeed(float max_speed);


      private:
        // acquired from user
        driver::MotorCANBase** motors_ = nullptr;
        chassis_model_t model_;

        // pids and current speeds for each motor on the chassis
        PowerLimit* power_limit_ = nullptr;
        float* speeds_ = nullptr;

        uint8_t wheel_num_ = 0;

        bool power_limit_on_ = false;
        power_limit_t power_limit_info_ = {
            .power_limit = 120,
            .WARNING_power = 108,
            .WARNING_power_buff = 50,
            .buffer_total_current_limit = 3500.0f * wheel_num_,
            .power_total_current_limit =
                5000.0f * wheel_num_ / 80.0f * power_limit_info_.power_limit,
        };

        float current_chassis_power_ = 0;
        float current_chassis_power_buffer_ = 0;

        float chassis_offset_;

        uint8_t can_bridge_tx_id_ = 0x00;
        float can_bridge_vx_ = 0;
        float can_bridge_vy_ = 0;
        float can_bridge_vt_ = 0;

        bool chassis_enable_ = true;

        bool has_super_capacitor_ = false;
        bool super_capacitor_enable_ = false;
        driver::SuperCap* super_capacitor_ = nullptr;

        float max_motor_speed_ = 2 * PI * 10;

    };

    class ChassisCanBridgeSender {
      public:
        ChassisCanBridgeSender(communication::CanBridge* can_bridge, uint8_t rx_id);
        void SetChassisRegId(uint8_t xy_reg_id, uint8_t turn_on_reg_id, uint8_t power_limit_reg_id,
                             uint8_t current_power_reg_id);
        void Enable();
        void Disable();
        /**
         * @brief set the speed for chassis motors
         *
         * @param x_speed chassis speed on x-direction
         * @param y_speed chassis speed on y-direction
         * @param turn_speed chassis clockwise turning speed
         */
        void SetSpeed(const float x_speed, const float y_speed = 0, const float turn_speed = 0);

        /**
         * @brief set the power limit for chassis motors
         * @param power_limit_on whether to enable power limit
         * @param power_limit total power limit, in [W]
         * @param chassis_power Current chassis power, in [W]
         * @param chassis_power_buffer Current chassis power buffer, in [J]
         */
        void SetPower(bool power_limit_on, float power_limit, float chassis_power,
                      float chassis_power_buffer, bool enable_supercap = false,
                      bool force_update = false);


      private:
        communication::CanBridge* can_bridge_;
        bool chassis_enable_ = true;
        bool chassis_power_limit_on_ = false;
        bool chassis_super_capacitor_enable_ = false;
        float chassis_power_limit_ = 120;
        uint8_t device_rx_id_ = 0x00;
        uint8_t chassis_xy_reg_id_ = 0x00;
        uint8_t chassis_turn_on_reg_id_ = 0x00;
        uint8_t chassis_power_limit_reg_id_ = 0x00;
        uint8_t chassis_current_power_reg_id_ = 0x00;
        communication::can_bridge_ext_id_t rx_id_;
        communication::can_bridge_data_t data_;

    };

}  // namespace control
