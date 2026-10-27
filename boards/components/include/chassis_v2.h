/*###########################################################
 # Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

/**
 * @file chassis_v2.h
 * @brief 底盘抽象父类、运动指令与状态定义，以及具体底盘控制接口
 * @details 控制实现位于 chassis_v2.cpp，纯运动学计算位于 chassis_kinematics.h。
 *          CAN Bridge 协议与收发由 chassis_can_bridge 独立处理。
 */

#pragma once

#include "DjiMotorBase.h"
#include "supercap.h"

namespace control {
    class NewPowerLimit;

    /**
     * @brief 麦轮与四全向轮的电机索引
     * @note 顺序固定为前左、前右、后左、后右；不限制其他底盘的执行器数量。
     */
    struct chassis_v2_wheel {
        /** @brief 四轮电机排列及数量 */
        enum {
            front_left,   ///< 前左轮，索引 0
            front_right,  ///< 前右轮，索引 1
            back_left,    ///< 后左轮，索引 2
            back_right,   ///< 后右轮，索引 3
            count         ///< 四轮电机数量
        };
    };

    /**
     * @brief 下发给底盘的目标运动指令
     * @details 沿用旧版 Chassis::SetSpeed 的等效轮速控制量和电机方向约定。
     *          X 向右、Y 向前、turn 顺时针为正，具体轮速由各底盘独立解算。
     * @note x/y 不是车体速度 [m/s]，turn 不是车体角速度 [rad/s]，不能直接与实测反馈相减。
     *       指令中不包含使能状态；启停通过 chassis_v2::Enable / chassis_v2::Disable 控制。
     */
    struct chassis_speed {
        float x = 0;     ///< X 方向目标控制量，向右为正
        float y = 0;     ///< Y 方向目标控制量，向前为正
        float turn = 0;  ///< 旋转目标控制量，顺时针为正
    };

    /**
     * @brief 底盘功率限制参数
     * @note enabled 只控制功率限制，不代表底盘或电机已经使能。
     */
    struct chassis_power {
        bool enabled = false;        ///< 功率限制开关
        float max_watt = 0;          ///< 功率预算 [W]，启用功率限制时应为有限正值
        float voltage = 0;           ///< 供电电压 [V]，当前四轮实现要求有限且大于 1
        uint8_t buffer_percent = 0;  ///< 缓冲能量百分比 [%]，UpdatePower 写入时限制到 [0, 100]
    };

    /**
     * @brief 底盘实际运行状态与实测运动反馈
     * @details enabled 是运行使能状态，反馈速度由应用的轮速采集或 IMU 等模块提供，不能复制目标指令。
     *          控制类内部只维护使能状态，不自动填充反馈速度及接收时间。
     * @note 反馈使用物理单位：X 向右、Y 向前，角速度逆时针为正，与 chassis_speed 的指令含义不同。
     */
    struct chassis_state {
        bool enabled = false;   ///< 底盘实际运行使能状态
        float feedback_vx = 0;  ///< 实测 X 方向速度 [m/s]，向右为正
        float feedback_vy = 0;  ///< 实测 Y 方向速度 [m/s]，向前为正
        float feedback_vt = 0;  ///< 实测偏航角速度 [rad/s]，逆时针为正
        /**
         * @brief 反馈在本机的接收时间 [ms]，使用 HAL_GetTick 的时间基准
         * @note 通信接收端取两路反馈中较早的接收时刻，不是远端采样时间或持续时长。
         *       0 是合法 tick；是否收到完整且新鲜的数据应检查通信层 ReadState 的返回值。
         */
        uint32_t time = 0;
    };

    // Physical XY velocity from measured actuators; invalid results contain zeros.
    struct chassis_feedback {
        float vx = 0;  // m/s, right positive
        float vy = 0;  // m/s, forward positive
        bool valid = false;
    };

    /**
     * @brief 底盘控制抽象父类
     * @details 保存运动指令、运行状态、功率参数和超级电容指针，提供公共启停与参数更新。
     *          具体底盘必须重写全部纯虚接口，负责自身执行器、目标解算及功率控制。
     *          父类不限定轮数，不持有电机组，也不负责 CAN Bridge 通信或传感器融合。
     * @note 控制与销毁应在同一控制任务中串行进行。输出前回调只在读取功率/使能快照时使用短临界区，
     *       不代表全部接口具备多任务并发保护。
     * @see mecanum
     * @see omni
     */
    class chassis_v2 {
      public:
        /**
         * @brief 通过父类接口析构具体底盘
         * @note 父类析构不调用纯虚函数；子类必须在释放执行器资源前完成停车。
         *       销毁前应先注销或替换引用本对象的电机输出前回调。
         */
        virtual ~chassis_v2() = default;

        /** @brief 禁止复制，避免多个对象共享同一组执行器与控制状态 */
        chassis_v2(const chassis_v2&) = delete;

        /** @brief 禁止复制赋值，保持执行器与底盘对象的唯一关联 */
        chassis_v2& operator=(const chassis_v2&) = delete;

        /**
         * @brief 检查具体底盘是否具备运行条件
         * @return true 表示允许使能；false 表示不能运行。
         * @note 子类定义就绪条件；此接口不保证自动检查电机在线状态、通信超时或 IMU 校准状态。
         */
        virtual bool IsReady() const = 0;

        /**
         * @brief Return measured chassis XY velocity, independent of enable/target state.
         * @note Each model samples its actual actuators and applies forward kinematics.
         *       Missing/stale/non-finite measurements or invalid geometry return {0,0,false}.
         *       Does not update chassis_state or perform IMU fusion/CAN transmission.
         */
        virtual chassis_feedback GetFeedback() const = 0;

        /**
         * @brief 检查并保存目标运动指令，准备执行器目标
         * @param x X 方向控制量，向右为正，使用旧版等效轮速单位
         * @param y Y 方向控制量，向前为正，默认 0
         * @param turn 旋转控制量，顺时针为正，默认 0
         * @return true 表示指令已接受；false 表示指令非法或底盘不就绪，子类应禁用并清除目标。
         * @note 不自动使能电机。子类负责写入 speed_；具体目标通过 Update 下发。
         */
        virtual bool SetSpeed(float x, float y = 0, float turn = 0) = 0;

        /**
         * @brief 执行一次底盘控制更新
         * @details 子类检查 state_ 和 IsReady，读取 speed_，结合当前反馈解算并下发执行器目标。
         *          底盘未使能、失去运行条件或解算失败时，应调用 Disable 停车。
         * @note 由应用的控制任务周期调用；state_.enabled 为 false 时不得启用执行器。
         */
        virtual void Update() = 0;

        /**
         * @brief 停车并清除目标、使能状态和执行器输出
         * @details 子类应清除 speed_、关闭 state_.enabled，禁用执行器和超级电容并将输出清零。
         * @note 必须允许重复调用。Disable 会调用此接口，因此实现中不能再次调用 Disable。
         */
        virtual void Stop() = 0;

        /**
         * @brief 设置具体底盘的电机速度上限
         * @param max_speed 电机目标速度绝对值上限；当前麦轮/全向轮使用输出轴角速度 [rad/s]
         * @return true 表示上限和当前目标已更新；false 表示参数非法、底盘不就绪或该型号不支持。
         * @note 当前四轮实现要求有限且大于 0，失败时禁用底盘；更新上限不会自动使能。
         */
        virtual bool SetMaxMotorSpeed(float max_speed) = 0;

        /**
         * @brief 在电机输出前执行具体底盘的功率限制
         * @details 子类从 power_/state_ 读取一致快照，根据自身执行器配置调整输出。
         *          底盘未使能时应清零输出；单独关闭功率限制开关不等于停车。
         * @note 应在电机 PID 计算后、CAN 输出前调用，通常由 ApplyPowerLimitWrapper 转发。
         */
        virtual void ApplyPowerLimit() = 0;

        /**
         * @brief 请求使能底盘并开启关联超级电容
         * @note IsReady 返回 false 时转为 Disable；本函数不直接下发电机目标，须继续调用 Update。
         */
        void Enable();

        /**
         * @brief 禁用底盘，清除运动指令并调用子类 Stop
         * @note 同时禁用关联超级电容；再次 Enable 不恢复之前的目标，可重复调用。
         */
        void Disable();

        /**
         * @brief 查询底盘运行使能状态
         * @return true 表示底盘已使能；false 表示底盘已禁用。
         * @note 仅反映控制状态，不代表电机在线或已经跟踪到目标速度。
         */
        bool IsEnabled() const;

        /**
         * @brief 更新功率限制参数，不直接修改电机输出
         * @param enabled 是否启用功率限制，与底盘运行使能独立
         * @param max_watt 功率预算 [W]，由具体底盘在输出前验证
         * @param voltage 电压 [V]；仅在有限且大于 1 时覆盖已保存值，否则保留本地电压
         * @param buffer_percent 缓冲能量百分比 [%]，大于 100 时按 100 保存
         * @note 使用保留 PRIMASK 的短临界区更新。实际限制由 ApplyPowerLimit 执行。
         */
        void UpdatePower(bool enabled, float max_watt, float voltage, uint8_t buffer_percent);

        /**
         * @brief 直接更新本地供电电压
         * @param voltage 本地采样电压 [V]
         * @note 本接口原样保存，不检查有效性；当前四轮实现会在功率限制阶段拒绝非法电压并清零输出。
         */
        void UpdatePowerVoltage(float voltage);

        /**
         * @brief 电机输出前回调入口，虚函数分派至具体底盘的 ApplyPowerLimit
         * @param args 有效且非空的 chassis_v2 指针，以 void* 形式传入
         * @note 由应用通过 driver::DjiMotorBase::RegisterPreOutputCallback 显式注册；不自动注册或注销。
         *       同一全局回调槽需要由应用统一管理，对象销毁前必须解除引用。
         */
        static void ApplyPowerLimitWrapper(void* args);

      protected:
        /**
         * @brief 初始化底盘公共数据
         * @param super_capacitor 外部持有的超级电容对象，nullptr 表示不使用；使用期间必须保持有效
         * @note 不取得超级电容所有权，也不调用纯虚函数；子类完成自身初始化后负责将执行器置于禁用状态。
         */
        explicit chassis_v2(driver::SuperCap* super_capacitor = nullptr);

        driver::SuperCap* super_capacitor_;  ///< 关联超级电容，不由底盘释放，可为 nullptr
        chassis_speed speed_;                ///< 最近接受的目标指令，不是实测反馈
        chassis_state state_;                ///< 运行状态，反馈字段由应用采集模块负责填充
        chassis_power power_;                ///< 最近保存的功率限制参数
    };

    /**
     * @brief 麦克纳姆轮底盘控制类
     * @details 直接继承 chassis_v2，独立管理四个 DJI 电机、轮速目标与四 M3508 参数的功率限制器。
     *          解算调用 kinematics::MecanumInverse，保持旧控制量、电机方向和限速行为。
     * @note 电机、速度环 PID、OMEGA 模式及减速比由应用配置；本类不自动检查电机在线或 IMU 状态。
     */
    class mecanum : public chassis_v2 {
      public:
        /**
         * @brief 麦克纳姆轮底盘配置
         * @note 配置内容在构造时复制；电机对象由应用持有，并须在底盘整个生命周期内有效。
         */
        struct config {
            driver::DjiMotorBase* motors[chassis_v2_wheel::count] = {};  ///< 前左、前右、后左、后右，指针非空且互异
            float offset = 0;                     ///< 无量纲前后轮旋转项偏置，分别使用 1-offset 和 1+offset
            float max_motor_speed = 2 * PI * 10;  ///< 电机输出轴目标速度上限 [rad/s]，默认 2π×10，必须大于 0
            float wheel_radius = 0;  ///< 轮半径 [m]，由 program 配置；未配置时反馈无效
        };

        /**
         * @brief 构造麦克纳姆轮底盘并将执行器置于禁用状态
         * @param config 电机排列、旋转偏置与电机速度上限
         * @param super_capacitor 外部超级电容对象，默认为 nullptr
         * @note 电机为空、重复或速度上限不大于 0 时触发断言；构造时创建功率限制器。
         *       不取得电机和超级电容的所有权，也不自动注册输出前回调。
         */
        explicit mecanum(const config& config, driver::SuperCap* super_capacitor = nullptr);

        /**
         * @brief 禁用底盘并释放内部功率限制器
         * @note 不释放电机或超级电容；调用者须先解除输出前回调对本对象的引用。
         */
        ~mecanum() override;

        /** @brief 禁止复制具有执行器和功率限制器关联的底盘对象 */
        mecanum(const mecanum&) = delete;

        /** @brief 禁止复制赋值，避免执行器冲突和功率限制器重复释放 */
        mecanum& operator=(const mecanum&) = delete;

        /**
         * @brief 查询当前麦克纳姆轮实现的就绪状态
         * @return 当前实现始终返回 true，构造阶段已经检查电机配置。
         * @note 不包含电机在线、通信超时或 IMU 检查，应用须自行处理这些条件。
         */
        bool IsReady() const override;

        chassis_feedback GetFeedback() const override;

        /**
         * @brief 保存指令并解算、等比例限制四轮速度目标
         * @param x X 方向旧版等效轮速控制量，向右为正
         * @param y Y 方向旧版等效轮速控制量，向前为正，默认 0
         * @param turn 旧版旋转控制量，顺时针为正，默认 0
         * @return true 表示指令已保存；false 表示输入/解算结果非有限或底盘不就绪，并已禁用底盘。
         * @note 不使能或下发电机目标；speed_ 保存原始指令，限速作用于解算后的轮速目标。
         */
        bool SetSpeed(float x, float y = 0, float turn = 0) override;

        /**
         * @brief 使用保存的指令刷新四轮目标并使能电机
         * @note 未使能或不就绪时 Disable；重新解算失败时也会禁用。
         *       本函数只向驱动设置目标，不执行电机 PID 或直接发送 CAN。
         */
        void Update() override;

        /**
         * @brief 清除指令和使能状态，禁用四轮与超级电容并清零目标及输出
         * @note 可重复调用；不会再次调用 Disable，避免递归。
         */
        void Stop() override;

        /**
         * @brief 更新电机速度上限并重新解算当前目标
         * @param max_speed 有限且大于 0 的电机输出轴速度上限 [rad/s]
         * @return true 表示上限及当前目标已更新；false 表示参数/当前目标非法或不就绪，并已禁用。
         * @note 不自动使能电机。
         */
        bool SetMaxMotorSpeed(float max_speed) override;

        /**
         * @brief 根据功率预算限制四轮输出电流
         * @details 临界区内快照读取功率与使能状态；未使能或不就绪时清零输出。
         *          运行中若关闭功率限制，则保持驱动计算的输出；预算、电压或转速非法时清零输出。
         * @note 使用现有四 M3508 功率模型。须在电机 PID 后、CAN 输出前调用。
         */
        void ApplyPowerLimit() override;

      private:
        /**
         * @brief 调用 kinematics::MecanumInverse 更新四轮未限速目标
         * @param command 旧版等效轮速运动指令
         * @note 只计算目标，不校验输入、不限速，也不操作电机；由 SetSpeed 组织检查及限速。
         */
        void Solve(const chassis_speed& command);

        /** @brief 清零四个电机的输出电流，不改变指令、目标或使能状态 */
        void ZeroOutputs();

        driver::DjiMotorBase* motors_[chassis_v2_wheel::count] = {};  ///< 外部电机指针，顺序为前左、前右、后左、后右
        float speeds_[chassis_v2_wheel::count] = {};                  ///< 四轮输出轴速度目标 [rad/s]
        float wheel_radius_;                                          ///< 本车型轮半径 [m]
        float offset_;                                                ///< 从配置复制的旋转项偏置系数
        float max_motor_speed_;                                       ///< 电机输出轴速度上限 [rad/s]
        NewPowerLimit* limiter_;                                      ///< 本对象创建和释放的四电机功率限制器
    };

    /**
     * @brief 四全向轮底盘控制类
     * @details 直接继承 chassis_v2，独立管理四个 DJI 电机、轮速目标与四 M3508 参数的功率限制器。
     *          解算调用 kinematics::OmniInverse，保持旧控制量、电机方向和限速行为。
     * @note 电机、速度环 PID、OMEGA 模式及减速比由应用配置；本类不自动检查电机在线或 IMU 状态。
     */
    class omni : public chassis_v2 {
      public:
        /**
         * @brief 四全向轮底盘配置
         * @note 配置内容在构造时复制；电机对象由应用持有，并须在底盘整个生命周期内有效。
         */
        struct config {
            driver::DjiMotorBase* motors[chassis_v2_wheel::count] = {};  ///< 前左、前右、后左、后右，指针非空且互异
            float offset = 0;                     ///< 无量纲前后轮旋转项偏置，分别使用 1-offset 和 1+offset
            float max_motor_speed = 2 * PI * 10;  ///< 电机输出轴目标速度上限 [rad/s]，默认 2π×10，必须大于 0
            float wheel_radius = 0;  ///< 轮半径 [m]，由 program 配置；未配置时反馈无效
        };

        /**
         * @brief 构造四全向轮底盘并将执行器置于禁用状态
         * @param config 电机排列、旋转偏置与电机速度上限
         * @param super_capacitor 外部超级电容对象，默认为 nullptr
         * @note 电机为空、重复或速度上限不大于 0 时触发断言；构造时创建功率限制器。
         *       不取得电机和超级电容的所有权，也不自动注册输出前回调。
         */
        explicit omni(const config& config, driver::SuperCap* super_capacitor = nullptr);

        /**
         * @brief 禁用底盘并释放内部功率限制器
         * @note 不释放电机或超级电容；调用者须先解除输出前回调对本对象的引用。
         */
        ~omni() override;

        /** @brief 禁止复制具有执行器和功率限制器关联的底盘对象 */
        omni(const omni&) = delete;

        /** @brief 禁止复制赋值，避免执行器冲突和功率限制器重复释放 */
        omni& operator=(const omni&) = delete;

        /**
         * @brief 查询当前四全向轮实现的就绪状态
         * @return 当前实现始终返回 true，构造阶段已经检查电机配置。
         * @note 不包含电机在线、通信超时或 IMU 检查，应用须自行处理这些条件。
         */
        bool IsReady() const override;

        chassis_feedback GetFeedback() const override;

        /**
         * @brief 保存指令并解算、等比例限制四轮速度目标
         * @param x X 方向旧版等效轮速控制量，向右为正
         * @param y Y 方向旧版等效轮速控制量，向前为正，默认 0
         * @param turn 旧版旋转控制量，顺时针为正，默认 0
         * @return true 表示指令已保存；false 表示输入/解算结果非有限或底盘不就绪，并已禁用底盘。
         * @note 不使能或下发电机目标；speed_ 保存原始指令，限速作用于解算后的轮速目标。
         */
        bool SetSpeed(float x, float y = 0, float turn = 0) override;

        /**
         * @brief 使用保存的指令刷新四轮目标并使能电机
         * @note 未使能或不就绪时 Disable；重新解算失败时也会禁用。
         *       本函数只向驱动设置目标，不执行电机 PID 或直接发送 CAN。
         */
        void Update() override;

        /**
         * @brief 清除指令和使能状态，禁用四轮与超级电容并清零目标及输出
         * @note 可重复调用；不会再次调用 Disable，避免递归。
         */
        void Stop() override;

        /**
         * @brief 更新电机速度上限并重新解算当前目标
         * @param max_speed 有限且大于 0 的电机输出轴速度上限 [rad/s]
         * @return true 表示上限及当前目标已更新；false 表示参数/当前目标非法或不就绪，并已禁用。
         * @note 不自动使能电机。
         */
        bool SetMaxMotorSpeed(float max_speed) override;

        /**
         * @brief 根据功率预算限制四轮输出电流
         * @details 临界区内快照读取功率与使能状态；未使能或不就绪时清零输出。
         *          运行中若关闭功率限制，则保持驱动计算的输出；预算、电压或转速非法时清零输出。
         * @note 使用现有四 M3508 功率模型。须在电机 PID 后、CAN 输出前调用。
         */
        void ApplyPowerLimit() override;

      private:
        /**
         * @brief 调用 kinematics::OmniInverse 更新四轮未限速目标
         * @param command 旧版等效轮速运动指令
         * @note 只计算目标，不校验输入、不限速，也不操作电机；由 SetSpeed 组织检查及限速。
         */
        void Solve(const chassis_speed& command);

        /** @brief 清零四个电机的输出电流，不改变指令、目标或使能状态 */
        void ZeroOutputs();

        driver::DjiMotorBase* motors_[chassis_v2_wheel::count] = {};  ///< 外部电机指针，顺序为前左、前右、后左、后右
        float speeds_[chassis_v2_wheel::count] = {};                  ///< 四轮输出轴速度目标 [rad/s]
        float wheel_radius_;  ///< 本车型轮半径 [m]
        float offset_;                                                ///< 从配置复制的旋转项偏置系数
        float max_motor_speed_;                                       ///< 电机输出轴速度上限 [rad/s]
        NewPowerLimit* limiter_;                                      ///< 本对象创建和释放的四电机功率限制器
    };

    /**
     * @brief steer 底盘接口占位
     * @details 尚未定义执行器布局、转向反馈及运动学，不持有电机或功率限制器。
     * @note 始终不就绪，不能使能；完成机械配置后须同时实现运行、停车和功率处理。
     */
    class steer : public chassis_v2 {
      public:
        /**
         * @brief 查询占位底盘的就绪状态
         * @return 始终返回 false。
         */
        bool IsReady() const override;

        chassis_feedback GetFeedback() const override;

        /**
         * @brief 拒绝运动指令并保持禁用
         * @param x 未使用的 X 方向控制量
         * @param y 未使用的 Y 方向控制量，默认 0
         * @param turn 未使用的旋转控制量，默认 0
         * @return 始终返回 false。
         */
        bool SetSpeed(float x, float y = 0, float turn = 0) override;

        /** @brief 保持底盘禁用，不执行运动控制 */
        void Update() override;

        /** @brief 清除公共指令/使能状态并关闭关联超级电容；当前无执行器需要操作 */
        void Stop() override;

        /**
         * @brief 拒绝速度上限设置并保持禁用
         * @param max_speed 未使用的电机速度上限
         * @return 始终返回 false。
         */
        bool SetMaxMotorSpeed(float max_speed) override;

        /** @brief 功率限制占位；当前无执行器，因此不执行任何操作 */
        void ApplyPowerLimit() override;
    };

    /**
     * @brief swerve 底盘接口占位
     * @details 尚未定义执行器布局、转向反馈及运动学，不持有电机或功率限制器。
     * @note 始终不就绪，不能使能；完成机械配置后须同时实现运行、停车和功率处理。
     */
    class swerve : public chassis_v2 {
      public:
        /**
         * @brief 查询占位底盘的就绪状态
         * @return 始终返回 false。
         */
        bool IsReady() const override;

        chassis_feedback GetFeedback() const override;

        /**
         * @brief 拒绝运动指令并保持禁用
         * @param x 未使用的 X 方向控制量
         * @param y 未使用的 Y 方向控制量，默认 0
         * @param turn 未使用的旋转控制量，默认 0
         * @return 始终返回 false。
         */
        bool SetSpeed(float x, float y = 0, float turn = 0) override;

        /** @brief 保持底盘禁用，不执行运动控制 */
        void Update() override;

        /** @brief 清除公共指令/使能状态并关闭关联超级电容；当前无执行器需要操作 */
        void Stop() override;

        /**
         * @brief 拒绝速度上限设置并保持禁用
         * @param max_speed 未使用的电机速度上限
         * @return 始终返回 false。
         */
        bool SetMaxMotorSpeed(float max_speed) override;

        /** @brief 功率限制占位；当前无执行器，因此不执行任何操作 */
        void ApplyPowerLimit() override;
    };

}  // namespace control
