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

#include "DjiMotorBase.h"
#include "pid.h"
#include "utils.h"

namespace control {

    /**
     * @brief 双 yaw 云台的电机偏移角度、最大角度、死区等参数
     * @note 上/下 yaw 是同轴串联的两级关节，各自独立标定，不能共用一套参数
     * @note 除了 max 由用户根据机械限位决定，其他都应该通过读取编码器值通过uart/gdb获取
     */
    /**
     * @brief offset, max, deadband and other constants of a dual yaw gimbal
     * @note upper / lower yaw are two coaxial joints in series, each needs its own
     * calibration; they must not share one set of yaw parameters
     * @note except for max is determined by user (mechanical limit), these should be
     * obtained by reading encoder values through uart/gdb
     */
    struct gimbal_DoubleYaw_data_t {
        /* pitch 轴 */
        /**
         * @brief 底盘水平且枪口水平时的 pitch 输出轴编码器角度，单位 rad。
         * @note 在水平标定时设置，运行中的 IMU 姿态补偿不修改此值。
         */
        float pitch_offset_ = 0.0f;
        /**
         * @brief pitch 的对称俯仰限幅，单位 rad；同时限制输入目标和相对 offset 的机械行程。
         * @note 上下行程不对称时取两侧安全行程的较小值。当前最短路径电机模式要求
         *       0 <= pitch_max_ < PI / 2，避免两端之间的运动被映射到限位外的短路径。
         */
        float pitch_max_ = 0.0f;
        /** @brief 是否反转 TargetAbs/TargetRel 的 pitch 输入；不改变 pitch 编码器或 IMU 反馈方向。 */
        bool pitch_inverted = false;
        float pitch_deadband = 0;  /* pitch 死区，小于该角度不动作 */

        /* 小yaw，靠近枪口的一级，其角度是相对下 yaw 的 */
        float upper_yaw_offset_ = 0.0f; /* upper yaw offset angle (angle when muzzle is at
                                  horizontal center) */
        float upper_yaw_max_ = 0.0f;    /* 上yaw关节相对下yaw的机械行程上限（如 90° = PI/2）；
                                           注意这是关节限位，不是云台朝向的限幅 */
        bool upper_yaw_circle_ = true;  /* upper yaw angle can circle or not */
        /**
         * @brief 小 yaw 编码器方向是否与云台 IMU yaw 正方向相反。
         * @details 固定底盘和大 yaw，若小 yaw 编码器增大时 IMU yaw 减小，置 true；
         *          两者同向增大则置 false。标定时避开角度跨圈点。
         *          此参数同时用于编码器到关节角、关节运动增量到电机目标的方向转换。
         * @note 不反转 TargetAbs/TargetRel 的 yaw 输入；调用方必须按 IMU yaw 正方向传入目标。
         */
        bool upper_yaw_inverted = false;
        float upper_yaw_deadband = 0;  /* upper yaw 死区，小于该角度不动作 */

        /* 大yaw，靠近底盘的一级，其角度是相对车身的 */
        float lower_yaw_offset_ = 0.0f; /* lower yaw offset angle (angle when muzzle is at
                                  horizontal center) */
        /**
         * @brief 大 yaw 相对机械中心的最大偏转角，单位 rad；连续旋转时不使用。
         * @note 有限行程使用最近的等价机械中心，要求 0 <= lower_yaw_max_ < PI。
         *       有限多圈机构需另行建立回零和圈数基准，本接口不支持。
         */
        float lower_yaw_max_ = 0.0f;
        bool lower_yaw_circle_ = true;  /* lower yaw angle can circle or not */
        /**
         * @brief 大 yaw 编码器方向是否与云台 IMU yaw 正方向相反。
         * @details 固定底盘和小 yaw，若大 yaw 编码器增大时 IMU yaw 减小，置 true；
         *          两者同向增大则置 false。标定时避开角度跨圈点。
         *          此参数同时用于编码器到关节角、关节运动增量到电机目标的方向转换，
         *          与小 yaw 的安装方向独立标定。
         * @note 不反转 TargetAbs/TargetRel 的 yaw 输入；调用方必须按 IMU yaw 正方向传入目标。
         */
        bool lower_yaw_inverted = false;
        float lower_yaw_deadband = 0;  /* lower yaw 死区，小于该角度不动作 */

    };

    /**
     * @brief 双 yaw 云台结构体
     */
    /**
     * @brief structure used when dual yaw gimbal instance is initialized
     */
    typedef struct {
        driver::DjiMotorBase* pitch_motor;      /* pitch motor instance           */
        driver::DjiMotorBase* upper_yaw_motor;  /* 上yaw / 小yaw 电机（靠近枪口） */
        driver::DjiMotorBase* lower_yaw_motor;  /* 下yaw / 大yaw 电机（靠近底盘） */
        gimbal_DoubleYaw_data_t data;            /* gimbal related constants       */
    } gimbal_DoubleYaw_t;

    /**
     * @brief 双 yaw 云台类
     * @details 控制 pitch、上yaw(小yaw)、下yaw(大yaw) 三个电机
     *
     * @details 大小yaw是同轴串联的两级关节：底盘 -> 下yaw(大yaw) -> 上yaw(小yaw) -> 枪口，
     *          因此"云台相对地面的朝向 = 下yaw关节角 + 上yaw关节角"。IMU 装在云台(上yaw输出)上，
     *          imu_yaw 直接就是该地面朝向。
     *
     * @details 两轴同周期响应：上yaw直接修正朝向误差，下yaw直接接手 u + 朝向误差。
     *          上yaw根据下yaw实际运动持续补偿并逐渐回中；两轴分别遵守关节限位。
     *          快慢响应由电机位置/速度环及速度输出限幅决定，不在云台层规划步长。
     */
    /**
     * @brief wrapper class for dual yaw gimbal
     */
    class Gimbal_DoubleYaw {
      public:
        /**
         * @brief 构造函数
         *
         * @param gimbal 用于初始化云台的结构体，参考gimbal_DoubleYaw_t
         */
        /**
         * @brief constructor for gimbal
         *
         * @param gimbal structure that used to initialize gimbal, refer to type
         * gimbal_DoubleYaw_t
         */
        Gimbal_DoubleYaw(gimbal_DoubleYaw_t gimbal);

        /**
         * @brief 析构函数
         */
        /**
         * @brief destructor for gimbal
         */
        ~Gimbal_DoubleYaw();

        /**
         * @brief 获取云台相关常量
         *
         * @return 参考gimbal_DoubleYaw_data_t
         */
        /**
         * @brief get gimbal related constants
         *
         * @return refer to gimbal_DoubleYaw_data_t
         */
        gimbal_DoubleYaw_data_t* GetData();

        /**
         * @brief 计算当前云台的输出
         * @note 不带IMU的开环模式：用两个yaw编码器的关节角之和作为当前朝向，
         *       同时完成大小yaw协调：上yaw吃误差、下yaw补差额并回中
         * @note pitch 目标在此模式解释为相对 pitch_offset_ 的关节角，不能补偿底盘倾斜。
         */
        /**
         * @brief calculate the output of the motors under current configuration
         * @note open loop mode, no IMU involved
         */
        void UpdateEncoder();

        /**
         * @brief 初始化/IMU校准时，将三个关节分别回到各自的标定零点
         * @note 同时将目标朝向清零；不使用正常跟踪的目标分配和朝向死区。
         *       必须在电机反馈有效后调用，并在回中/校准期间持续调用。
         * @note 仅返回已有标定位置，不根据 IMU 自动校准 pitch_offset_；底盘倾斜时不保证枪口水平。
         */
        void Recenter();

        /**
         * @brief 基于当前传感器数据更新云台的输出
         * @param imu_pitch_angle 陀螺仪测量的pitch角度，范围为[-pi, pi]
         * @note pitch IMU 随枪口俯仰，水平为零，正方向须由调用方统一为编码器增大的方向。
         *       机械限幅只限制电机目标，不改写保存的对地 pitch 目标或 pitch_offset_。
         * @param imu_yaw_angle 陀螺仪测量的yaw角度，范围为[-pi, pi]
         * @note IMU 需要装在云台(上yaw输出)上，此时 imu_yaw 就是枪口相对地面的朝向；
         *       若IMU装在底盘上，本类的增量闭环会失去稳定点（每周期都被命令再转一个固定角）
         */
        /**
         * @brief update the output of the motors based on current sensor data
         * @param imu_pitch_angle pitch angle measured by gyroscope, range is [-pi, pi]
         * @param imu_yaw_angle yaw angle measured by gyroscope, range is [-pi, pi]
         */
        void UpdateIMU(float imu_pitch_angle, float imu_yaw_angle);

        /**
         * @brief 设置绝对目标；IMU 模式下 pitch 相对水平、yaw 相对 IMU 朝向零点。
         * @param new_pitch 新的pitch角度
         * @param new_yaw 新的yaw角度
         * @note pitch 输入应用 pitch_inverted 后按 ±pitch_max_ 饱和，0 表示水平，不环绕。
         * @note new_yaw 是"云台相对地面的目标朝向"，不受 upper_yaw_max_ 限幅
         *       （±90° 是上yaw关节相对下yaw的行程，超过90°的朝向由大小yaw协调实现）
         * @note new_yaw 与传入 UpdateIMU 的 imu_yaw_angle 同正方向；yaw 反转参数仅描述关节方向，
         *       输入设备的方向转换由调用方完成。
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
         * @note new_yaw 是按 IMU yaw 正方向定义的目标增量，不受 yaw 反转参数影响。
         * @note pitch 增量应用 pitch_inverted 后累加并保存饱和值，持续同向输入不会绕圈。
         */
        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetRel(float new_pitch, float new_yaw);

        /**
         * @brief 用增量更新云台的标定偏移量。
         *
         * @param pitch_offset_delta pitch offset 的增量，单位 rad。
         * @param upper_yaw_offset 上yaw(小yaw) offset 的增量，单位 rad。
         * @param lower_yaw_offset 下yaw(大yaw) offset 的增量，单位 rad。
         * @note pitch 的对地目标保持不变；绝对设置水平标定值应使用 SetPitchOffset。
         */
        /**
         * @brief update the offset of the gimbal
         *
         * @param pitch_offset_delta pitch offset increment
         * @param upper_yaw_offset upper yaw offset increment
         * @param lower_yaw_offset lower yaw offset increment
         */
        void UpdateOffset(float pitch_offset_delta, float upper_yaw_offset, float lower_yaw_offset);

        /**
         * @brief 绝对设置水平标定得到的 pitch 编码器零点，不改变对地 pitch 目标。
         * @param encoder_at_level 底盘与枪口均水平且稳定时的输出轴编码器角度，单位 rad。
         * @note 调用方负责确认标定姿态及反馈有效性；仅更新内存，不下发电机目标、不持久化。
         *       禁止根据运行中的 IMU 误差持续更新；更新后机械限位随新零点重新解释。
         */
        void SetPitchOffset(float encoder_at_level);

        /** @brief 获取应用输入反转后的有符号 pitch 目标，单位 rad；水平为零，不包含 offset。 */
        float getPitchTarget() const;

        /* 上yaw(小yaw) 是主控轴，它保存的是"目标朝向"而不是关节角；
           真实目标朝向 = 返回值 - data_.upper_yaw_offset_ */
        float getUpperYawTarget() const;

        /* 下yaw(大yaw) 跟随上yaw，一般不单独作为朝向参考；返回下yaw的目标角度（编码器域） */
        float getLowerYawTarget() const;

        /**
         * @brief 获取相对水平标定零点的 pitch 关节角，单位 rad，范围 [-PI, PI]。
         * @note 消除累计反馈的整圈偏置；底盘倾斜时此值不等于对地俯仰。
         */
        float getPitchByMotor() const;

        /*
         * @brief 通过电机编码器 获取 上yaw(小yaw)相对标定零点(即相对下yaw)的关节角，
         *        带符号、正方向 = 地面朝向增大的方向
         */
        float getUpperYawByMotor() const;

        /**
         * @brief 获取大 yaw 相对标定零点的关节角，正方向与云台 IMU yaw 一致。
         * @return 连续旋转模式返回多圈累计角；有限行程模式返回 [-PI, PI] 内的关节角，
         *         消除累计反馈相对标定零点的整圈偏置。单位 rad。
         */
        float getLowerYawByMotor() const;

      private:
        /**
         * @brief 大小yaw协调核心：把本周期需要修正的朝向误差分配给两个yaw轴，并直接下发目标
         * @param yaw_diff 本周期需要修正的地面朝向误差 [rad]
         * @note 两轴同周期直接接收位置目标；上yaw根据实际反馈补偿，不提前让角。
         *       快慢响应由各电机的位置/速度环决定，两轴分别遵守关节限位。
         */
        void CoordinateYaw(float yaw_diff);

        driver::DjiMotorBase* pitch_motor_ = nullptr;     /* pitch          */
        driver::DjiMotorBase* upper_yaw_motor_ = nullptr; /* 上yaw / 小yaw  */
        driver::DjiMotorBase* lower_yaw_motor_ = nullptr; /* 下yaw / 大yaw  */

        // pitch and yaw constants
        gimbal_DoubleYaw_data_t data_;

        // pitch and yaw angle
        float pitch_target_ = 0.0f; /* 有符号对地俯仰目标，水平为零，不含 offset、不环绕 */
        float upper_yaw_angle_; /* 上yaw是主控轴，这里存的是目标朝向（电机域），
                                   真实目标朝向 = upper_yaw_angle_ - upper_yaw_offset_ */
        float lower_yaw_angle_; /* 下yaw(大yaw)的目标角度（编码器域），由 Update* 计算 */

    };

}  // namespace control
