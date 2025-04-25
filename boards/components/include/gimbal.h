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
#include "pid.h"
#include "utils.h"

namespace control {

    /**
     * @brief 不同云台的电机偏移角度、最大角度、最小角度
     * @note 除了最小角度由用户决定，其他都应该通过读取编码器值通过uart/gdb获取
     */
    /**
     * @brief offset, max, and proximity angles of different gimbals
     * @note except for proximity is determined by user, these should be obtained
     * by reading encoder values through uart/gdb
     */
    struct gimbal_data_t {
        float pitch_offset_ = 0.0f; /* pitch offset angle (angle when muzzle is at vertical
                                  center) */
        float yaw_offset_ = 0.0f;   /* yaw offset angle (angle when muzzle is at horizontal
                                  center) */
        float pitch_max_ = 0.0f;    /* maximum pitch angle the gimbal can turn from center */
        float yaw_max_ = 0.0f;      /* maximum yaw angle the gimbal can turn from center      */
        bool yaw_circle_ = true;    /* yaw angle can circle or not */
        bool pitch_inverted = false;
        bool yaw_inverted = false;
        float pitch_eposition = 0;
        float yaw_eposition = 0;
    };

    /**
     * @brief 标准云台结构体
     */
    /**
     * @brief structure used when gimbal instance is initialized
     */
    typedef struct {
        driver::MotorCANBase* pitch_motor; /* pitch motor instance */
        driver::MotorCANBase* yaw_motor;   /* yaw motor instance   */
        gimbal_data_t data;                /* gimbal related constants */
    } gimbal_t;

    /**
     * @brief 云台类
     * @details 用于控制云台，标准云台类的实现是控制两个6020电机
     */
    /**
     * @brief wrapper class for gimbal
     */
    class Gimbal {
      public:
        /**
         * @brief 构造函数
         *
         * @param gimbal 用于初始化云台的结构体，参考gimbal_t
         */
        /**
         * @brief constructor for gimbal
         *
         * @param gimbal structure that used to initialize gimbal, refer to type
         * gimbal_t
         */
        Gimbal(gimbal_t gimbal);

        /**
         * @brief 析构函数
         */
        /**
         * @brief destructor for gimbal
         */
        ~Gimbal();

        /**
         * @brief 获取云台相关常量
         *
         * @return 参考gimbal_data_t
         */
        /**
         * @brief get gimbal related constants
         *
         * @return refer to gimbal_data_t
         */
        gimbal_data_t* GetData();

        /**
         * @brief 计算当前云台的输出
         * @note 不会立即控制电机
         */
        /**
         * @brief calculate the output of the motors under current configuration
         * @note does not command the motor immediately
         */
        void Update();

        /**
         * @brief 基于当前传感器数据更新云台的输出
         * @param pitch 陀螺仪测量的pitch角度，范围为[-pi, pi]
         * @param yaw 陀螺仪测量的yaw角度，范围为[-pi, pi]
         */
        /**
         * @brief update the output of the motors based on current sensor data
         * @param pitch pitch angle measured by gyroscope, range is [-pi, pi]
         * @param yaw yaw angle measured by gyroscope, range is [-pi, pi]
         */
        void UpdateIMU(float pitch, float yaw);

        /**
         * @brief 将云台指向新的方向，是绝对于车身零点的角度
         * @param new_pitch 新的pitch角度
         * @param new_yaw 新的yaw角度
         */
        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetAbs(float new_pitch, float new_yaw);

        /**
         * @brief 将云台指向新的方向，是相对于当前目标方向的角度
         *
         * @param new_pitch 新的pitch角度
         * @param new_yaw 新的yaw角度
         */
        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetRel(float new_pitch, float new_yaw);

        /**
         * @brief 将云台指向新的方向，是相对于当前云台方向的角度
         * @param new_pitch
         * @param new_yaw
         */
        void TargetReal(float new_pitch, float new_yaw);

        /**
         * @brief 更新云台的偏移量
         *
         * @param pitch_offset 新的pitch偏移量
         * @param yaw_offset 新的yaw偏移量
         */
        /**
         * @brief update the offset of the gimbal
         *
         * @param pitch_offset new pitch offset
         * @param yaw_offset   new yaw offset
         */
        void UpdateOffset(float pitch_offset, float yaw_offset);

        float getPitchTarget() const;
        float getYawTarget() const;

        /*
         * @brief 通过电机编码器 获取 云台相对于云台参数标定的零点 的pitch
         */
        float getPitchByMotor() const;
        /*
         * @brief 通过电机编码器 获取 云台相对于云台参数标定的零点 的yaw
         */
        float getYawByMotor() const;

      private:
        // acquired from user
        driver::MotorCANBase* pitch_motor_ = nullptr;
        driver::MotorCANBase* yaw_motor_ = nullptr;

        // pitch and yaw constants
        gimbal_data_t data_;

        // pitch and yaw angle
        float pitch_angle_; /* current gimbal pitch angle */
        float yaw_angle_;   /* current gimbal yaw angle   */

        // pitch and yaw limit
        float pitch_lower_limit_; /* pitch lower limit */
        float pitch_upper_limit_; /* pitch upper limit */
        float yaw_lower_limit_;   /* yaw lower limit   */
        float yaw_upper_limit_;   /* yaw upper limit   */
    };

}  // namespace control
