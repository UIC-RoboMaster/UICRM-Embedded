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
    struct dual_yaw_gimbal_data_t {
        /* pitch 轴 */
        float pitch_offset_ = 0.0f; /* pitch offset angle (angle when muzzle is at vertical
                                  center) */
        float pitch_max_ = 0.0f;    /* maximum pitch angle the gimbal can turn from center */
        bool pitch_inverted = false;
        float pitch_deadband = 0;  /* pitch 死区，小于该角度不动作 */

        /* 小yaw，靠近枪口的一级，其角度是相对下 yaw 的 */
        float upper_yaw_offset_ = 0.0f; /* upper yaw offset angle (angle when muzzle is at
                                  horizontal center) */
        float upper_yaw_max_ = 0.0f;    /* 上yaw关节相对下yaw的机械行程上限（如 90° = PI/2）；
                                           注意这是关节限位，不是云台朝向的限幅 */
        bool upper_yaw_circle_ = true;  /* upper yaw angle can circle or not */
        bool upper_yaw_inverted = false; /* 输入(遥控/自瞄)的yaw正方向与云台朝向正方向相反时置 true，
                                            只影响 Target* 的输入方向，和编码器极性无关 */
        float upper_yaw_deadband = 0;  /* upper yaw 死区，小于该角度不动作 */

        /* 大yaw，靠近底盘的一级，其角度是相对车身的 */
        float lower_yaw_offset_ = 0.0f; /* lower yaw offset angle (angle when muzzle is at
                                  horizontal center) */
        float lower_yaw_max_ = 0.0f;    /* maximum lower yaw angle it can turn from center */
        bool lower_yaw_circle_ = true;  /* lower yaw angle can circle or not */
        bool lower_yaw_inverted = false;
        float lower_yaw_deadband = 0;  /* lower yaw 死区，小于该角度不动作 */

        /* 大小yaw协调：上yaw为主控快轴，下yaw慢速跟随并把上yaw收回中心 */
        /* 关节角正方向标定：电机编码器增大时，该级关节是否让"云台相对地面的朝向"同向增大。
           地面朝向增量 = 上yaw关节角增量 * su + 下yaw关节角增量 * sl
           （IMU 装在云台(上yaw输出)上，直接测得该地面朝向）
           电机反装、编码器反接时置 true，标定方法见 programs/Seer/README.md */
        bool upper_yaw_joint_inverted = false;
        bool lower_yaw_joint_inverted = false;
        /* 下yaw跟随步长 [rad]：每个控制周期下yaw朝"最终由它承担的朝向 θ*"最多走该角度，
           上yaw同时补上剩余误差（两者之和 = 本周期朝向误差，朝向由大小yaw之和/IMU闭环保持）。
           这样从上电第一周期起两个yaw就都在转：上yaw快、下yaw慢，下yaw逐步把上yaw的角度接手，
           上yaw回到中心。等效交接角速度 ≈ 该值 / 下yaw位置环时间常数（与电机PID有关，需实测），
           例如位置环时间常数 20ms、该值取 1.2° 时约 60°/s。
           取 0 表示下yaw不主动接手（只补上yaw顶行程限位后吃不下的差额）。
           注意交接越快，上yaw已经把角度让出去、下yaw还没跟上造成的短暂朝向偏差越大
           （量级约等于该值），所以要按现场实测的精度要求来定，不要一味加大 */
        float lower_yaw_recenter_max_step = 0.0f;
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
        dual_yaw_gimbal_data_t data;            /* gimbal related constants       */
    } dual_yaw_gimbal_t;

    /**
     * @brief 双 yaw 云台类
     * @details 控制 pitch、上yaw(小yaw)、下yaw(大yaw) 三个电机
     *
     * @details 大小yaw是同轴串联的两级关节：底盘 -> 下yaw(大yaw) -> 上yaw(小yaw) -> 枪口，
     *          因此"云台相对地面的朝向 = 下yaw关节角 + 上yaw关节角"。IMU 装在云台(上yaw输出)上，
     *          imu_yaw 直接就是该地面朝向。
     *
     * @details 协调策略（上yaw为主控快轴，下yaw为慢轴跟随）：
     *          1. 下yaw从第一周期起就朝"最终由它承担的朝向 θ*"走，但每周期按
     *             lower_yaw_recenter_max_step 限速（慢轴）；上yaw同时补上剩余误差
     *             （快轴），两者之和 = 本周期朝向误差，朝向由大小yaw之和/IMU闭环保持；
     *          2. 上yaw被夹在自己的关节行程内（典型 ±90°，由 upper_yaw_max_ /
     *             upper_yaw_circle_ 决定）；顶到行程边界后吃不下的差额由下yaw补上
     *             （此时"只依靠大yaw转"）；下yaw自身若有限位也会被夹住；
     *          3. 随着下yaw把角度接手，上yaw同步收回中心，最终上yaw回到中心、
     *             下yaw承担全部角度，上yaw重新获得左右两侧的快速权限。
     */
    /**
     * @brief wrapper class for dual yaw gimbal
     */
    class Dual_Yaw_Gimbal {
      public:
        /**
         * @brief 构造函数
         *
         * @param gimbal 用于初始化云台的结构体，参考dual_yaw_gimbal_t
         */
        /**
         * @brief constructor for gimbal
         *
         * @param gimbal structure that used to initialize gimbal, refer to type
         * dual_yaw_gimbal_t
         */
        Dual_Yaw_Gimbal(dual_yaw_gimbal_t gimbal);

        /**
         * @brief 析构函数
         */
        /**
         * @brief destructor for gimbal
         */
        ~Dual_Yaw_Gimbal();

        /**
         * @brief 获取云台相关常量
         *
         * @return 参考dual_yaw_gimbal_data_t
         */
        /**
         * @brief get gimbal related constants
         *
         * @return refer to dual_yaw_gimbal_data_t
         */
        dual_yaw_gimbal_data_t* GetData();

        /**
         * @brief 计算当前云台的输出
         * @note 不带IMU的开环模式：用两个yaw编码器的关节角之和作为当前朝向，
         *       同时完成大小yaw协调：上yaw吃误差、下yaw补差额并回中
         */
        /**
         * @brief calculate the output of the motors under current configuration
         * @note open loop mode, no IMU involved
         */
        void UpdateEncoder();

        /**
         * @brief 基于当前传感器数据更新云台的输出
         * @param imu_pitch_angle 陀螺仪测量的pitch角度，范围为[-pi, pi]
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
         * @brief 将云台指向新的方向，是绝对于车身零点的角度
         * @param new_pitch 新的pitch角度
         * @param new_yaw 新的yaw角度
         * @note new_yaw 是"云台相对地面的目标朝向"，不受 upper_yaw_max_ 限幅
         *       （±90° 是上yaw关节相对下yaw的行程，超过90°的朝向由大小yaw协调实现）
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
         * @param upper_yaw_offset 新的上yaw(小yaw)偏移量
         * @param lower_yaw_offset 新的下yaw(大yaw)偏移量
         */
        /**
         * @brief update the offset of the gimbal
         *
         * @param pitch_offset     new pitch offset
         * @param upper_yaw_offset new upper yaw offset
         * @param lower_yaw_offset new lower yaw offset
         */
        void UpdateOffset(float pitch_offset, float upper_yaw_offset, float lower_yaw_offset);

        float getPitchTarget() const;
        /* 上yaw(小yaw) 是主控轴，它保存的是"目标朝向"而不是关节角；
           真实目标朝向 = 返回值 - data_.upper_yaw_offset_ */
        float getUpperYawTarget() const;
        /* 下yaw(大yaw) 跟随上yaw，一般不单独作为朝向参考；返回下yaw的目标角度（编码器域） */
        float getLowerYawTarget() const;

        /*
         * @brief 通过电机编码器 获取 云台相对于云台参数标定的零点 的pitch
         */
        float getPitchByMotor() const;
        /*
         * @brief 通过电机编码器 获取 上yaw(小yaw)相对标定零点(即相对下yaw)的关节角，
         *        带符号、正方向 = 地面朝向增大的方向
         */
        float getUpperYawByMotor() const;
        /*
         * @brief 通过电机编码器 获取 下yaw(大yaw)相对标定零点的关节角（可多圈累计），
         *        带符号、正方向 = 地面朝向增大的方向
         */
        float getLowerYawByMotor() const;

      private:
        /**
         * @brief 大小yaw协调核心：把本周期需要修正的朝向误差分配给两个yaw轴，并直接下发目标
         * @param yaw_diff 本周期需要修正的地面朝向误差 [rad]
         * @note 下yaw(大yaw)从第一周期起就朝最终目标走（按 lower_yaw_recenter_max_step 限速），
         *       上yaw(小yaw)补上剩余误差并夹在自己的关节行程内，两者之和 = yaw_diff
         */
        void CoordinateYaw(float yaw_diff);

        // acquired from user
        driver::DjiMotorBase* pitch_motor_ = nullptr;     /* pitch          */
        driver::DjiMotorBase* upper_yaw_motor_ = nullptr; /* 上yaw / 小yaw  */
        driver::DjiMotorBase* lower_yaw_motor_ = nullptr; /* 下yaw / 大yaw  */

        // pitch and yaw constants
        dual_yaw_gimbal_data_t data_;

        // pitch and yaw angle
        float pitch_angle_;     /* 目标pitch角（电机域，含 pitch_offset_） */
        float upper_yaw_angle_; /* 上yaw是主控轴，这里存的是目标朝向（电机域），
                                   真实目标朝向 = upper_yaw_angle_ - upper_yaw_offset_ */
        float lower_yaw_angle_; /* 下yaw(大yaw)的目标角度（编码器域），由 Update* 计算 */

        // pitch limit
        float pitch_lower_limit_; /* pitch lower limit */
        float pitch_upper_limit_; /* pitch upper limit */
    };

}  // namespace control
