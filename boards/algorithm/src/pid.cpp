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

#include "pid.h"

#include "utils.h"

namespace control {

    int16_t ClipMotorRange(float output) {
        constexpr int MIN = -MOTOR_RANGE; /* Minimum that a 16-bit number can represent */
        constexpr int MAX = MOTOR_RANGE;  /* Maximum that a 16-bit number can represent */
        return (int16_t)clip<int>((int)output, MIN, MAX);
    }

    typedef struct {
        float kp;
        float ki;
        float kd;
    } pid_t;

    PIDController::PIDController() {
        pid_f32_.Kp = 0;
        pid_f32_.Ki = 0;
        pid_f32_.Kd = 0;
        arm_pid_init_f32(&pid_f32_, 1);
    }

    PIDController::PIDController(float kp, float ki, float kd) {
        pid_f32_.Kp = kp;
        pid_f32_.Ki = ki;
        pid_f32_.Kd = kd;
        arm_pid_init_f32(&pid_f32_, 1);
    }

    PIDController::PIDController(float* param) : PIDController(param[0], param[1], param[2]) {
    }

    float PIDController::ComputeOutput(float error) {
        return arm_pid_f32(&pid_f32_, error);
    }

    int16_t PIDController::ComputeConstrainedOutput(float error) {
        /*
         * CAN protocal uses a 16-bit signed number to drive the motors, so this
         * version of the output computation can make sure that no unexpected
         * behavior (overflow) can happen.
         */
        return control::ClipMotorRange(arm_pid_f32(&pid_f32_, error));
    }

    void PIDController::Reinit(float kp, float ki, float kd) {
        pid_f32_.Kp = kp;
        pid_f32_.Ki = ki;
        pid_f32_.Kd = kd;
        arm_pid_init_f32(&pid_f32_, 0);
    }

    void PIDController::Reinit(float* param) {
        Reinit(param[0], param[1], param[2]);
    }

    void PIDController::Reset() {
        arm_pid_init_f32(&pid_f32_, 1);
    }

    ConstrainedPID::ConstrainedPID() {
        Reinit(0, 0, 0, 0, 0);
        Reset();
        ChangeMax(0, 0);
        mode_ = Integral_Limit;
    }

    ConstrainedPID::ConstrainedPID(float kp, float ki, float kd, float max_iout, float max_out) {
        Reinit(kp, ki, kd, max_iout, max_out);
        Reset();
        mode_ = Integral_Limit;
    }

    ConstrainedPID::ConstrainedPID(float* param, float max_iout, float max_out) {
        Reinit(param[0], param[1], param[2], max_iout, max_out);
        Reset();
        mode_ = Integral_Limit;
    }

    ConstrainedPID::ConstrainedPID(ConstrainedPID::PID_Init_t pid_init) {
        Reinit(pid_init);
        Reset();
        ResetIntegral();
    }

    float ConstrainedPID::ComputeOutput(float target, float measure) {
        if (mode_ & ErrorHandle) {
            // 异常处理
            PID_ErrorHandle();
            if (PID_ErrorHandler.error_type != PID_ERROR_NONE) {
                // 发现问题，则调用回调函数
                error_callback_(error_callback_instance_, PID_ErrorHandler);
                // 清除问题
                PID_ErrorHandler.error_type = PID_ERROR_NONE;
                PID_ErrorHandler.error_count = 0;
                return 0;
            }
        }

        // 更新目标值和当前的测量值
        target_ = target;
        measure_ = measure;
        error_ = target_ - measure_;

        // 如果误差小于死区，则不进行处理
        if (abs(error_) > dead_band_) {
            // 比死区大，则进行pid计算
            pout_ = kp_ * error_;
            // 此处的iterm_仅仅计算了当前的积分值，累积积分值后续会进行计算
            iterm_ = ki_ * error_;
            dout_ = kd_ * (error_ - last_error_);

            // 梯形积分计算
            if (mode_ & Trapezoid_Intergral) {
                PID_TrapezoidIntegral();
            }

            // 变速积分，参考此文章中的变速积分计算
            // https://www.cnblogs.com/WangHongxi/p/12409382.html
            if (mode_ & ChangingIntegralRate) {
                PID_ChangingIntegralRate();
            }

            // 积分限幅
            if (mode_ & Integral_Limit) {
                PID_IntegralLimit();
            }

            // 将微分的参考值转为对实际测量值的参考而不是对误差的参考，避免突然修改error导致的微分爆炸
            if (mode_ & Derivative_On_Measurement) {
                PID_DerivativeOnMeasurement();
            }

            // 微分滤波，采取当前值和上一次的值的加权平均
            if (mode_ & DerivativeFilter) {
                PID_DerivativeFilter();
            }

            // 计算输出值
            iout_ += iterm_;
            output_ = pout_ + iout_ + dout_;

            // 计算输出滤波，采取当前值和上一次的值的加权平均
            if (mode_ & OutputFilter) {
                PID_OutputFilter();
            }

            // 输出限幅
            PID_OutputLimit();

            // 微分限幅
            PID_ProportionLimit();
        } else {
            iout_ = 0;
            pout_ = 0;
            dout_ = 0;
            output_ = 0;
        }

        // 将本次计算的值保存，用于下一次计算
        last_measure_ = measure_;
        last_output_ = output_;
        last_dout_ = dout_;
        last_error_ = error_;
        return output_;
    }

    int16_t ConstrainedPID::ComputeConstrainedOutput(float error) {
        return (int16_t)ComputeOutput(error);
    }

    void ConstrainedPID::Reinit(float kp, float ki, float kd, float max_iout, float max_out) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        if (ki_ == 0) {
            iout_ = 0;
        }
        ChangeMax(max_iout, max_out);
    }

    void ConstrainedPID::Reinit(float* param, float max_iout, float max_out) {
        Reinit(param[0], param[1], param[2], max_iout, max_out);
    }

    void ConstrainedPID::Reset() {
        pout_ = 0;
        iout_ = 0;
        dout_ = 0;
        error_ = 0;
        last_error_ = 0;
    }
    void ConstrainedPID::ChangeMax(float max_iout, float max_out) {
        max_iout_ = max_iout;
        max_out_ = max_out;
    }
    ConstrainedPID::PID_State_t ConstrainedPID::State() const {
        return {
            .error = error_,
            .pout = pout_,
            .iout = iout_,
            .dout = dout_,
            .output = output_,
        };
    }

    void ConstrainedPID::PID_ErrorHandle() {
        // pid错误处理，目前用于判断电机堵转
        // 请在速度环使用电机堵转判断
        if (output_ < max_out_ * 0.01f) {
            // 排除PID输出本身很小的情况
            return;
        }
        // 电机是否难以移动，此处的意思是当实际速度几乎等于0的时候，电机无法转动
        if (abs(target_ - measure_) / target_ > 0.9f) {
            PID_ErrorHandler.error_count++;
        } else {
            PID_ErrorHandler.error_count = 0;
            PID_ErrorHandler.error_type = PID_ERROR_NONE;
        }

        // 检测到连续1s电机无法转动，则认为电机堵转
        if (PID_ErrorHandler.error_count > 1000) {
            // 1s 堵转了
            PID_ErrorHandler.error_type = Motor_Blocked;
        }
    }
    void ConstrainedPID::PID_ChangingIntegralRate() {
        if (error_ * iout_ > 0) {
            // 符号相同
            if (abs(error_) <= ScalarB) {
                return;  // 满了
            }
            if (abs(error_) <= (ScalarA + ScalarB)) {
                iterm_ *= (ScalarA + ScalarB - abs(error_)) / ScalarA;
            } else {
                iterm_ = 0;
            }
        }
    }
    void ConstrainedPID::PID_TrapezoidIntegral() {
        iterm_ = ki_ * (error_ + last_error_) / 2;
    }
    void ConstrainedPID::PID_IntegralLimit() {
        float temp_Output, temp_Iout;
        temp_Iout = iout_ + iterm_;
        temp_Output = pout_ + iout_ + dout_;
        if (abs(temp_Output) > max_out_) {
            if (error_ * iout_ > 0) {
                // Integral still increasing
                iterm_ = 0;
            }
        }

        if (temp_Iout > max_iout_) {
            iterm_ = 0;
            iout_ = max_iout_;
        }
        if (temp_Iout < -max_iout_) {
            iterm_ = 0;
            iout_ = -max_iout_;
        }
    }
    void ConstrainedPID::PID_DerivativeOnMeasurement() {
        dout_ = kd_ * (measure_ - last_measure_);
    }
    void ConstrainedPID::PID_OutputFilter() {
        output_ = output_ * Output_Filtering_Coefficient +
                  last_output_ * (1 - Output_Filtering_Coefficient);
    }
    void ConstrainedPID::PID_OutputLimit() {
        output_ = clip<float>(output_, -max_out_, max_out_);
    }
    void ConstrainedPID::PID_DerivativeFilter() {
        dout_ = dout_ * Derivative_Filtering_Coefficient +
                last_dout_ * (1 - Derivative_Filtering_Coefficient);
    }
    void ConstrainedPID::PID_ProportionLimit() {
        pout_ = clip<float>(pout_, -max_out_, max_out_);
    }
    void ConstrainedPID::ResetIntegral() {
        iout_ = 0;
        iterm_ = 0;
    }
    void ConstrainedPID::Reinit(ConstrainedPID::PID_Init_t pid_init) {
        kp_ = pid_init.kp;
        ki_ = pid_init.ki;
        kd_ = pid_init.kd;
        iterm_ = 0;
        max_out_ = pid_init.max_out;
        max_iout_ = pid_init.max_iout;
        dead_band_ = pid_init.deadband;
        target_ = 0;

        ScalarA = pid_init.A;
        ScalarB = pid_init.B;

        Output_Filtering_Coefficient = pid_init.output_filtering_coefficient;
        Derivative_Filtering_Coefficient = pid_init.derivative_filtering_coefficient;

        mode_ = pid_init.mode;

        PID_ErrorHandler.error_count = 0;
        PID_ErrorHandler.error_type = PID_ERROR_NONE;

        output_ = 0;
    }
    void ConstrainedPID::RegisterErrorCallcack(ConstrainedPID::PID_ErrorCallback_t callback,
                                               void* instance) {
        error_callback_ = callback;
        error_callback_instance_ = instance;
    }

} /* namespace control */
