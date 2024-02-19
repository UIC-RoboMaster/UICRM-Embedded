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
        kp_ = pid_init.kp;
        ki_ = pid_init.ki;
        kd_ = pid_init.kd;
        iterm_=0;
        max_out_ = pid_init.max_out;
        max_iout_ = pid_init.max_iout;
        dead_band_ = pid_init.deadband;
        target_=0;

        ScalarA=pid_init.A;
        ScalarB=pid_init.B;

        Output_Filtering_Coefficient=pid_init.output_filtering_coefficient;
        Derivative_Filtering_Coefficient = pid_init.derivative_filtering_coefficient;

        mode_ = pid_init.mode;

        PID_ErrorHandler.error_count=0;
        PID_ErrorHandler.error_type=PID_ERROR_NONE;

        output_=0;
    }

    float ConstrainedPID::ComputeOutput(float measure,float target) {
        if(mode_ & ErrorHandle){
            //异常处理
            PID_ErrorHandle();
            if(PID_ErrorHandler.error_type!=PID_ERROR_NONE){
                //发现问题
                return 0;
            }
        }

        target_=target;
        measure_=measure;
        error_=target_-measure_;

        if(abs(error_)>dead_band_){
            //比死区大，处理
        }


//        if((error > 0 && cumulated_err_ < 0) || (error < 0 && cumulated_err_ > 0))
//            cumulated_err_ = 0;
        if (ki_ != 0) {
            cumulated_err_ += error;
            cumulated_err_ = clip<float>(cumulated_err_, -max_iout_ / ki_, max_iout_ / ki_);
        } else {
            cumulated_err_ = 0;
        }
        float out = kp_ * error + ki_ * cumulated_err_ + kd_ * (error - last_error_);
        out = clip<float>(out, -max_out_, max_out_);
        last_error_ = error;
        return out;
    }



    int16_t ConstrainedPID::ComputeConstrainedOutput(float error) {
        return (int16_t)ComputeOutput(error);
    }



    void ConstrainedPID::Reinit(float kp, float ki, float kd, float max_iout, float max_out) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        if(ki_==0){
            iout_=0;
        }
        ChangeMax(max_iout, max_out);
    }

    void ConstrainedPID::Reinit(float* param, float max_iout, float max_out) {
        Reinit(param[0], param[1], param[2], max_iout, max_out);
    }

    void ConstrainedPID::Reset() {
        cumulated_err_ = 0;
        last_error_ = 0;
    }
    void ConstrainedPID::ChangeMax(float max_iout, float max_out) {
        max_iout_ = max_iout;
        max_out_ = max_out;
    }
    void ConstrainedPID::PID_ErrorHandle() {
        if(output_<max_out_*0.01f){
            return;
        }
        if(abs(target_-measure_)/target_>0.9f){
            PID_ErrorHandler.error_count++;
        }
        else{
            PID_ErrorHandler.error_count=0;
        }

        if(PID_ErrorHandler.error_count>200){
            //200ms 堵转了
            PID_ErrorHandler.error_type=Motor_Blocked;
        }
    }

} /* namespace control */
