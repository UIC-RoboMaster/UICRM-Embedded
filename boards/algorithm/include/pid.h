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

#pragma once

/* NOTE(alvin): DSP libraries depends on macro definitions on FPU
 * computability, so the main.h must be included before arm_math.h */
// clang-format off
#include "main.h"
// clang-format on

#include "arm_math.h"

namespace control {

    /**
     * @brief DJI电机输出范围
     */
    /**
     * @brief range of DJI motor output
     */
    constexpr int MOTOR_RANGE = 30000;

    /**
     * @brief 将PID获得的浮点输出转换为DJI电机输出
     * @param output PID输出的浮点数
     * @return DJI电机的实际输出值
     */
    int16_t ClipMotorRange(float output);

    /**
     * @brief 基础PID控制器
     */
    /**
     * @brief simple PID controller
     */
    class PIDController {
      public:
        /**
         * @brief PID控制器默认构造函数
         */
        /**
         * @brief PID controller default constructor
         */
        PIDController();

        /**
         * @brief PID控制器构造函数
         * @param kp 比例增益
         * @param ki 积分增益
         * @param kd 微分增益
         */
        /**
         * @brief PID controller constructor
         *
         * @param kp proportional gain
         * @param ki integral gain
         * @param kd derivative gain
         */
        PIDController(float kp, float ki, float kd);

        /**
         * @brief PID控制器构造函数
         * @param param PID控制器的增益，格式为[kp, ki, kd]
         */
        /**
         * @brief PID controller constructor
         *
         * @param param gains of PID controller, formated as [kp, ki, kd]
         */
        PIDController(float* param);

        /**
         * @brief 根据当前误差计算输出
         *
         * @param error 系统的误差，即（目标值 - 实际值）
         * @return 可以将误差驱动到0的输出值
         */
        /**
         * @brief compute output base on current error
         *
         * @param error error of the system, i.e. (target - actual)
         *
         * @return output value that could potentially drive the error to 0
         */
        float ComputeOutput(float error);

        /**
         * @brief 根据当前误差计算输出，但输出值被限制在DJI电机的范围内（适用于DJI电机输出）
         *
         * @param error 系统的误差，即（目标值 - 实际值）
         *
         * @return 可以将误差驱动到0的输出值，被限制在-30000到30000之间
         */
        /**
         * @brief compute output base on current error but constraint to range of
         * the DJI motor
         *
         * @param error error of the system, i.e. (target - actual)
         *
         * @return output value that could potentially drive the error to 0,
         *         floored at -30000, ceiled at 30000
         */
        int16_t ComputeConstrainedOutput(float error);

        /**
         * @brief 重新初始化PID控制器，但不清除当前状态
         *
         * @param kp 新的比例增益
         * @param ki 新的积分增益
         * @param kd 新的微分增益
         */
        /**
         * @brief reinitialize the pid instance using another set of gains, but does
         * not clear current status
         *
         * @param kp new proportional gain
         * @param ki new integral gain
         * @param kd new derivative gain
         */
        void Reinit(float kp, float ki, float kd);

        /**
         * @brief 重新初始化PID控制器，但不清除当前状态
         *
         * @param param PID控制器的增益，格式为[kp, ki, kd]
         */
        /**
         * @brief reinitialize the pid instance using another set of gains, but does
         * not clear current status
         *
         * @param param gains of PID controller, formated as [kp, ki, kd]
         */
        void Reinit(float* param);

        /**
         * @brief 清除PID控制器的状态
         */
        /**
         * @brief clear the remembered states of the controller
         */
        void Reset();

      private:
        arm_pid_instance_f32 pid_f32_;
    };

    /**
     * @brief 带有积分输出限制的PID控制器
     */
     /**
      * @brief PID controller with integral output constraint
      */
    class ConstrainedPID {
      public:
        /**
         * @brief PID控制器默认构造函数
         */
         /**
          * @brief PID controller default constructor
          */
        ConstrainedPID();
        /**
         * @brief PID控制器构造函数
         *
         * @param kp 比例增益
         * @param ki 积分增益
         * @param kd 微分增益
         * @param max_iout 积分输出限制
         * @param max_out 输出限制
         */
        /**
         * @brief PID controller constructor
         *
         * @param kp proportional gain
         * @param ki integral gain
         * @param kd derivative gain
         * @param max_iout integral output constraint
         * @param max_out output constraint
         */
        ConstrainedPID(float kp, float ki, float kd, float max_iout, float max_out);

        /**
         * @brief PID控制器构造函数
         *
         * @param param PID控制器的增益，格式为[kp, ki, kd]
         * @param max_iout 积分输出限制
         * @param max_out 输出限制
         */
        /**
         * @brief PID controller constructor
         *
         * @param param gains of PID controller, formated as [kp, ki, kd]
         * @param max_iout integral output constraint
         * @param max_out output constraint
         */
        ConstrainedPID(float* param, float max_iout, float max_out);

        /**
         * @brief 根据当前误差计算输出
         *
         * @param error 系统的误差，即（目标值 - 实际值）
         *
         * @return 可以将误差驱动到0的输出值
         */
        /**
         * @brief compute output base on current error
         *
         * @param error   error of the system, i.e. (target - actual)
         *
         * @return output value that could potentially drive the error to 0
         */
        float ComputeOutput(float error);

        /**
         * @brief 根据当前误差计算输出，并且其他传感器已经给出准确微分值
         * @param error 系统的误差，即（目标值 - 实际值）
         * @param omega 系统的微分值
         * @return 可以将误差驱动到0的输出值
         */
         /**
          * @brief compute output base on current error and accurate derivative
          * @param error error of the system, i.e. (target - actual)
          * @param omega derivative of the system
          * @return output value that could potentially drive the error to 0
          */
        float ComputeOutputWithOmega(float error, float omega);

        /**
         * @brief 根据当前误差计算输出，但输出值被限制在DJI电机的范围内（适用于DJI电机输出）
         * @param error 系统的误差，即（目标值 - 实际值）
         * @return 可以将误差驱动到0的输出值，被限制在-30000到30000之间
         */
         /**
          * @brief compute output base on current error but constraint to range of
          * The DJI motor
          * @param error error of the system, i.e. (target - actual)
          * @return output value that could potentially drive the error to 0,
          *        floored at -30000, ceiled at 30000
          */
        int16_t ComputeConstrainedOutput(float error);

        /**
         * @brief 根据当前误差计算输出，但输出值被限制在DJI电机的范围内（适用于DJI电机输出），并且其他传感器已经给出准确微分值
         * @param error 系统的误差，即（目标值 - 实际值）
         * @param omega 系统的微分值
         * @return 可以将误差驱动到0的输出值，被限制在-30000到30000之间
         */
        int16_t ComputeConstrainedOutputWithOmega(float error, float omega);

        /**
         * @brief 重新初始化PID控制器，但不清除当前状态
         *
         * @param kp 新的比例增益
         * @param ki 新的积分增益
         * @param kd 新的微分增益
         *
         * @param max_iout 积分输出限制
         * @param max_out 输出限制
         */
        /**
         * @brief reinitialize the pid instance using another set of gains, but does
         * not clear current status
         *
         * @param kp new proportional gain
         * @param ki new integral gain
         * @param kd new derivative gain
         *
         * @param max_iout integral output constraint
         * @param max_out output constraint
         */
        void Reinit(float kp, float ki, float kd, float max_iout, float max_out);

        /**
         * @brief 重新初始化PID控制器，但不清除当前状态
         *
         * @param param PID控制器的增益，格式为[kp, ki, kd]
         *
         * @param max_iout 积分输出限制
         * @param max_out 输出限制
         */
        /**
         * @brief reinitialize the pid instance using another set of gains, but does
         * not clear current status
         *
         * @param param gains of PID controller, formated as [kp, ki, kd]
         *
         * @param max_iout integral output constraint
         * @param max_out output constraint
         */
        void Reinit(float* param, float max_iout, float max_out);

        /**
         * @brief 清除PID控制器的状态
         */
        /**
         * @brief clear the remembered states of the controller
         */
        void Reset();

        /**
         * @brief 修改PID控制器的输出限制
         * @param max_iout 积分输出限制
         * @param max_out 输出限制
         */
         /**
          * @brief change the output constraint of the controller
          * @param max_iout integral output constraint
          * @param max_out output constraint
          */
        void ChangeMax(float max_iout, float max_out);

        float kp_;
        float ki_;
        float kd_;

        float cumulated_err_;

      private:
        float last_err_;

        float max_iout_;
        float max_out_;
    };

} /* namespace control */
