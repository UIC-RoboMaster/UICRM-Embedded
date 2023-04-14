#pragma once

#include "can.h"
#include "controller.h"
#include "motor.h"
#include "utils.h"

namespace control {

    /**
     * @brief gimbal models
     */
    typedef enum { GIMBAL_FORTRESS, GIMBAL_SENTRY, GIMBAL_STEERING } gimbal_model_t;

    /**
     * @brief offset, max, and proximity angles of different gimbals
     * @note except for proximity is determined by user, these should be obtained
     * by reading encoder values through uart/gdb
     */
    typedef struct {
        float pitch_offset_; /* pitch offset angle (angle when muzzle is at vertical
                                center) */
        float yaw_offset_;   /* yaw offset angle (angle when muzzle is at horizontal
                                center) */
        float pitch_max_;    /* maximum pitch angle the gimbal can turn from center */
        float yaw_max_;      /* maximum yaw angle the gimbal can turn from center      */
    } gimbal_data_t;

    /**
     * @brief structure used when gimbal instance is initialized
     */
    typedef struct {
        MotorCANBase* pitch_motor; /* pitch motor instance */
        MotorCANBase* yaw_motor;   /* yaw motor instance   */
        gimbal_model_t model;      /* gimbal model         */
    } gimbal_t;

    /**
     * @brief wrapper class for gimbal
     */
    class Gimbal {
      public:
        /**
         * @brief constructor for gimbal
         *
         * @param gimbal structure that used to initialize gimbal, refer to type
         * gimbal_t
         */
        Gimbal(gimbal_t gimbal);

        /**
         * @brief destructor for gimbal
         */
        ~Gimbal();

        /**
         * @brief get gimbal related constants
         *
         * @return refer to gimbal_data_t
         */
        gimbal_data_t* GetData();

        /**
         * @brief calculate the output of the motors under current configuration
         * @note does not command the motor immediately
         */
        void Update();

        void UpdateIMU(float pitch, float yaw);

        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetAbs(float new_pitch, float new_yaw);

        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetRel(float new_pitch, float new_yaw);

        /**
         * @brief set motors to point to a new orientation
         *
         * @param abs_pitch new pitch max angle
         * @param abs_yaw   new yaw max angle
         */
        void TargetAbsWOffset(float abs_pitch, float abs_yaw);

        /**
         * @brief update the offset of the gimbal
         *
         * @param pitch_offset new pitch offset
         * @param yaw_offset   new yaw offset
         */
        void UpdateOffset(float pitch_offset, float yaw_offset);



      private:
        // acquired from user
        MotorCANBase* pitch_motor_ = nullptr;
        MotorCANBase* yaw_motor_ = nullptr;
        gimbal_model_t model_;

        // pitch and yaw constants
        gimbal_data_t data_;

        // pitch and yaw pid
        float* pitch_theta_pid_param_ =
            nullptr; /* pid param that used to control pitch motor when moving  */
        float* pitch_omega_pid_param_ =
            nullptr; /* pid param that used to control pitch motor when holding */
        float* yaw_theta_pid_param_ =
            nullptr; /* pid param that used to control yaw motor when moving */
        float* yaw_omega_pid_param_ =
            nullptr; /* pid param that used to control yaw motor when holding */
        ConstrainedPID* pitch_theta_pid_ = nullptr; /* pitch theta pid */
        ConstrainedPID* pitch_omega_pid_ = nullptr; /* pitch omega pid */
        ConstrainedPID* yaw_theta_pid_ = nullptr;   /* yaw theta pid   */
        ConstrainedPID* yaw_omega_pid_ = nullptr;   /* yaw omega pid   */

        // pitch and yaw angle
        float pitch_angle_; /* current gimbal pitch angle */
        float yaw_angle_;   /* current gimbal yaw angle   */

        // pitch and yaw limit
        float pitch_lower_limit_; /* pitch lower limit */
        float pitch_upper_limit_; /* pitch upper limit */
        float yaw_lower_limit_;   /* yaw lower limit   */
        float yaw_upper_limit_;   /* yaw upper limit   */

        // state detectors
        BoolEdgeDetector pitch_detector_; /* pitch pid mode toggle detector */
        BoolEdgeDetector yaw_detector_;   /* yaw pid mode toggle detector   */

    };

}  // namespace control
