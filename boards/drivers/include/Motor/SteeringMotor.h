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

#include "ServoMotor.h"

namespace driver {

    typedef bool (*align_detect_t)(void);

    /**
     * @brief 舵轮用转向电机的初始化结构体
     */
    typedef struct {
        DjiMotorBase* motor; /* motor instance to be wrapped as a servomotor      */
        float max_speed;     /* desired turning speed of motor shaft, in [rad/s]  */
        float test_speed;
        float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
        float transmission_ratio; /* transmission ratio of motor */
        float offset_angle;
        float* omega_pid_param; /* pid parameter used to control speed of motor */
        float max_iout;
        float max_out;
        align_detect_t align_detect_func;
        float calibrate_offset;
    } steering_t;

    /**
     * @brief 舵轮用转向电机的包装类，用于精确控制电机的角度，可以用于带有外部减速箱的电机
     *
     * @note 这个类现在还处于测试阶段，如果没有专业知识，请不要使用
     */
    class SteeringMotor {
      public:
        SteeringMotor(steering_t data);
        float GetRawTheta() const;
        /**
         * @brief print out motor data
         */
        void PrintData() const;
        void TurnRelative(float angle);
        void TurnAbsolute(float angle);
        bool AlignUpdate();
        void Update();

      private:
        ServoMotor* servo_;

        float test_speed_;
        align_detect_t align_detect_func;
        float calibrate_offset;

        float align_angle_;
        BoolEdgeDetector* align_detector;
        bool align_complete_;
    };

}  // namespace driver
