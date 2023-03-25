#include "controller.h"

#include "motor.h"
#include "utils.h"

namespace control {

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
         * version of the output computation can make sure that no unexpected behavior
         * (overflow) can happen.
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
        Reinit(0, 0, 0);
        Reset();
        ChangeMax(0, 0);
    }

    ConstrainedPID::ConstrainedPID(float kp, float ki, float kd, float max_iout, float max_out) {
        Reinit(kp, ki, kd, max_iout, max_out);
        Reset();
    }

    ConstrainedPID::ConstrainedPID(float* param, float max_iout, float max_out) {
        Reinit(param[0], param[1], param[2], max_iout, max_out);
        Reset();
    }

    float ConstrainedPID::ComputeOutput(float error) {
        cumulated_err_ += error;
        last_err_ = error;
        cumulated_err_ = clip<float>(cumulated_err_, -max_iout_ / ki_, max_iout_ / ki_);
        float out = kp_ * error + ki_ * cumulated_err_ + kd_ * (error - last_err_);
        out = clip<float>(out, -max_out_, max_out_);
        return out;
    }

    int16_t ConstrainedPID::ComputeConstrainedOutput(float error) {
        return control::ClipMotorRange(ComputeOutput(error));
    }

    void ConstrainedPID::Reinit(float kp, float ki, float kd, float max_iout, float max_out) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        ChangeMax(max_iout, max_out);
    }

    void ConstrainedPID::Reinit(float* param, float max_iout, float max_out) {
        Reinit(param[0], param[1], param[2], max_iout, max_out);
    }

    void ConstrainedPID::Reset() {
        cumulated_err_ = 0;
        last_err_ = 0;
    }
    void ConstrainedPID::ChangeMax(float max_iout, float max_out) {
        max_iout_ = max_iout;
        max_out_ = max_out;
    }

} /* namespace control */
