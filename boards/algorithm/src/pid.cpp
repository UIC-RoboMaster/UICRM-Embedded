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
                // 发现问题
                error_callback_(error_callback_instance_, PID_ErrorHandler);
                // 清除问题
                PID_ErrorHandler.error_type = PID_ERROR_NONE;
                PID_ErrorHandler.error_count = 0;
                return 0;
            }
        }

        target_ = target;
        measure_ = measure;
        error_ = target_ - measure_;

        if (abs(error_) > dead_band_) {
            // 比死区大，处理
            pout_ = kp_ * error_;
            iterm_ = ki_ * error_;
            dout_ = kd_ * (error_ - last_error_);

            // Trapezoid Intergral
            if (mode_ & Trapezoid_Intergral) {
                PID_TrapezoidIntegral();
            }
            // Changing Integral Rate

            if (mode_ & ChangingIntegralRate) {
                PID_ChangingIntegralRate();
            }
            // Integral limit

            if (mode_ & Integral_Limit) {
                PID_IntegralLimit();
            }
            // Derivative On Measurement

            if (mode_ & Derivative_On_Measurement) {
                PID_DerivativeOnMeasurement();
            }
            // Derivative filter

            if (mode_ & DerivativeFilter) {
                PID_DerivativeFilter();
            }
            iout_ += iterm_;
            output_ = pout_ + iout_ + dout_;
            // Output Filter

            if (mode_ & OutputFilter) {
                PID_OutputFilter();
            }
            // Output limit

            PID_OutputLimit();
            // Proportional limit

            PID_ProportionLimit();
        }

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
    void ConstrainedPID::PID_ErrorHandle() {
        if (output_ < max_out_ * 0.01f) {
            return;
        }
        if (abs(target_ - measure_) / target_ > 0.9f) {
            PID_ErrorHandler.error_count++;
        } else {
            PID_ErrorHandler.error_count = 0;
            PID_ErrorHandler.error_type = PID_ERROR_NONE;
        }

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
