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
 * @file chassis_can_bridge.h
 * @brief 云台与底盘之间的 CAN Bridge 协议、收发接口及缓存定义
 */

#pragma once

#include "can_bridge.h"
#include "chassis_v2.h"

namespace communication {
    /**
     * @brief 底盘协议寄存器地址
     * @details 每个寄存器使用一帧 8 字节 CAN 数据，布局如下（字节下标从 0 开始）：
     * | 地址 | CAN Bridge 类型 | 数据布局 |
     * | ---- | --------------- | -------- |
     * | 0x80 | TWO_UINT32 | 云台心跳：uint32 序号、uint32 运行时间 [ms] |
     * | 0x81 | TWO_UINT32 | 底盘心跳：uint32 序号、uint32 运行时间 [ms] |
     * | 0x82 | EIGHT_UINT8 | 0~5：x/y/turn 各占 2 字节；6：控制标志；7：保留 |
     * | 0x83 | EIGHT_UINT8 | 0~5：vx/vy/vt 各占 2 字节；6：电机在线掩码；7：状态标志 |
     * | 0x84 | TWO_FLOAT | 0：功率限制开关；1：功率 [W]；2：缓冲能量 [%]；3：保留；4~7：float 电压 [V] |
     *
     * 运动指令与反馈的三个分量均按 IEEE 754 半精度浮点数编码，低字节在前，同帧更新。
     * 控制标志 bit0 为底盘使能，bit1 为系统活动状态。
     * 电机在线掩码 bit0~7 对应最多 8 个电机；四轮底盘 bit0~3 依次为前左、前右、后左、后右。
     * 状态标志 bit0 为底盘使能，bit1 为 IMU 校准状态，bit2 为错误状态，bit3 为功率限制状态。
     * @note 0x84 虽使用 TWO_FLOAT 类型标签，前 4 字节实际为四个 uint8_t 字段。
     *       发送端将保留字节置零，接收端忽略保留字节。
     */
    enum class chassis_register : uint8_t {
        heartbeat_gimbal = 0x80,   ///< 云台心跳：发送序号及运行时间
        heartbeat_chassis = 0x81,  ///< 底盘心跳：发送序号及运行时间
        command = 0x82,            ///< 运动指令及控制标志
        feedback = 0x83,           ///< 实测运动反馈、电机在线掩码及状态标志
        power_limit = 0x84,        ///< 功率限制参数
    };

    /**
     * @brief 底盘 CAN Bridge 地址与接收超时配置
     * @note 超时以本机 HAL_GetTick 为基准，从最近一次有效接收开始计时。
     *       数据年龄大于对应阈值才判定过期；阈值为 0 时仅接收当毫秒有效，不表示禁用超时。
     */
    struct chassis_can_bridge_config {
        uint8_t local_address = 0;             ///< 本机 CAN Bridge 地址，必须非零
        uint8_t peer_address = 0;              ///< 对端 CAN Bridge 地址，必须非零；仅接收该地址发来的数据
        uint32_t command_timeout_ms = 100;     ///< 运动指令有效期 [ms]
        uint32_t feedback_timeout_ms = 50;     ///< 实测反馈有效期 [ms]
        uint32_t power_timeout_ms = 250;       ///< 功率参数有效期 [ms]
        uint32_t heartbeat_timeout_ms = 1000;  ///< 云台与底盘心跳各自的有效期 [ms]
    };

    /**
     * @brief 云台与底盘之间的 CAN 通信桥
     * @details 封装运动指令、功率参数、实测反馈和心跳的单帧收发。
     *          CAN 接收回调校验对端地址、数据类型及数值后更新缓存；Read 接口供任务上下文读取。
     *          每次读取在短临界区中复制快照并检查超时，退出时恢复原有中断屏蔽状态。
     * @note Read 接口不会消耗缓存，可在有效期内重复读取；未接收或过期时返回 false 并清零输出。
     *       各寄存器独立计时，心跳有效不代表运动指令有效；超时后的停车策略由应用负责。
     * @note 本对象持有内部 CanBridge，外部 CAN 对象由应用管理。同一 CAN 总线和本机地址只应使用一个桥。
     *       当前没有回调注销接口，CAN 接收期间必须保持本对象及 CAN 对象存活。
     */
    class chassis_can_bridge {
      public:
        /**
         * @brief 构造通信桥并注册本机地址及全部寄存器的接收回调
         * @param can 硬件 CAN 对象，不能为空，生命周期应覆盖通信桥的使用期
         * @param config 本机/对端地址及超时配置，构造时复制保存
         * @note CAN 指针为空或任一地址为 0 时触发断言。
         */
        chassis_can_bridge(bsp::CAN* can, const chassis_can_bridge_config& config);

        /** @brief 禁止复制，保持接收回调与通信桥实例的唯一关联 */
        chassis_can_bridge(const chassis_can_bridge&) = delete;

        /** @brief 禁止复制赋值，避免覆盖已注册回调对应的通信状态 */
        chassis_can_bridge& operator=(const chassis_can_bridge&) = delete;

        /**
         * @brief 向对端发送运动指令与控制标志
         * @param command 目标控制量，x 向右、y 向前、turn 顺时针为正，沿用 chassis_speed 的等效轮速约定
         * @param chassis_enable 底盘使能请求，写入控制标志 bit0
         * @param system_active 系统活动状态，写入控制标志 bit1
         * @note 任一分量为非有限值时不发送；半精度转换有精度损失且不钳位，调用方应限制幅值。
         *       指令不是以 m/s、rad/s 表示的车体实测速度。
         */
        void SendCommand(const control::chassis_speed& command, bool chassis_enable, bool system_active);

        /**
         * @brief 向对端发送功率限制参数
         * @param power 功率限制开关、功率预算 [W]、缓冲能量 [%] 及供电电压 [V]
         * @note max_watt 必须有限且在 [0, 255]，编码为 uint8_t 时舍去小数；buffer_percent 必须在 [0, 100]。
         *       voltage 必须有限且非负，0 约定使用底盘本地电压；任一参数非法时整帧不发送。
         *       enabled 仅表示功率限制开关，不代表底盘运行使能。
         */
        void SendPower(const control::chassis_power& power);

        /**
         * @brief 向对端发送实测运动反馈、电机在线掩码及状态标志
         * @param state 实测 vx/vy [m/s]、vt [rad/s] 及使能状态；X 向右、Y 向前、旋转逆时针为正
         * @param motors_online 电机在线位掩码，bit0~3 依次为前左、前右、后左、后右，最多支持 8 个电机
         * @param status_flags 状态标志，bit1 为 IMU 校准状态，bit2 为错误状态，bit3 为功率限制状态
         * @note bit0 始终由 state.enabled 覆盖，其余位保留；state.time 不参与传输。
         *       反馈必须由应用采集。任一速度分量为非有限值时不发送；半精度转换有精度损失且不钳位。
         */
        void SendFeedback(const control::chassis_state& state, uint8_t motors_online, uint8_t status_flags);

        /**
         * @brief 向对端发送心跳序号与本机运行时间
         * @param is_gimbal true 使用云台心跳寄存器，false 使用底盘心跳寄存器
         * @param uptime_ms 本机运行时间 [ms]，由调用方提供
         * @note 两类心跳共用从 0 开始的 uint32_t 发送序号，每次调用递增，溢出后回绕。
         */
        void SendHeartbeat(bool is_gimbal, uint32_t uptime_ms);

        /**
         * @brief 在任务上下文读取最近一次有效运动指令
         * @param[out] command 解码后的目标控制量，含义与 SendCommand 相同
         * @param[out] chassis_enable 对端发送的底盘使能请求
         * @param[out] system_active 对端发送的系统活动状态
         * @return true 表示已有指令且未超过 command_timeout_ms；false 表示未接收或已过期。
         * @note 返回 false 时三个控制量清零，两个标志均置为 false。
         */
        bool ReadCommand(control::chassis_speed& command, bool& chassis_enable, bool& system_active) const;

        /**
         * @brief 在任务上下文读取最近一次有效功率参数
         * @param[out] power 解码后的功率限制参数；voltage 为 0 时保留该值，由应用选择本地电压
         * @return true 表示已有参数且未超过 power_timeout_ms；false 表示未接收或已过期。
         * @note 返回 false 时 enabled 置为 false，其余字段清零。
         */
        bool ReadPower(control::chassis_power& power) const;

        /**
         * @brief 在任务上下文读取最近一次有效实测反馈
         * @param[out] state 实测速度及使能状态；time 为该帧在本机的接收时间 [ms]，使用 HAL_GetTick 基准
         * @param[out] motors_online 电机在线位掩码，位定义与 SendFeedback 相同
         * @param[out] status_flags 完整状态标志，bit0 与 state.enabled 一致
         * @return true 表示已有反馈且未超过 feedback_timeout_ms；false 表示未接收或已过期。
         * @note 返回 false 时 state 所有字段、在线掩码及状态标志均清零。
         */
        bool ReadFeedback(control::chassis_state& state, uint8_t& motors_online, uint8_t& status_flags) const;

        /**
         * @brief 在任务上下文读取指定来源的最近一次有效心跳
         * @param from_gimbal true 读取云台心跳，false 读取底盘心跳；两者均只接受配置的对端地址
         * @param[out] seq 对端发送的心跳序号
         * @param[out] uptime_ms 对端运行时间 [ms]，不是本机接收时间
         * @return true 表示已有对应心跳且未超过 heartbeat_timeout_ms；false 表示未接收或已过期。
         * @note 返回 false 时 seq 和 uptime_ms 均清零；接收端不校验序号连续性或单调性。
         */
        bool ReadHeartbeat(bool from_gimbal, uint32_t& seq, uint32_t& uptime_ms) const;

      private:
        /**
         * @brief CAN 接收回调入口，将数据转发至对应实例的 Receive
         * @param id CAN Bridge 扩展标识符
         * @param data 原始 8 字节数据
         * @param args 注册回调时传入的 chassis_can_bridge 实例指针
         */
        static void ReceiveWrapper(can_bridge_ext_id_t id, can_bridge_data_t data, void* args);

        /**
         * @brief 校验并解码接收帧，更新对应缓存及本机接收时间
         * @param id 包含发送地址、寄存器和类型的 CAN Bridge 扩展标识符
         * @param data 原始 8 字节数据
         * @note 由 CAN 接收回调调用；非配置对端、类型错误或数值非法的数据被丢弃，不刷新有效期。
         */
        void Receive(can_bridge_ext_id_t id, can_bridge_data_t data);

        /**
         * @brief 通过内部 CanBridge 向配置的对端发送一帧数据
         * @param reg 目标寄存器
         * @param type CAN Bridge 数据类型标签
         * @param data 已编码的 8 字节负载
         */
        void Send(chassis_register reg, can_bridge_type_e type, can_bridge_data_t data);

        /**
         * @brief 将单精度浮点数转换为 IEEE 754 半精度位表示
         * @param f 待编码的单精度浮点数
         * @return 半精度浮点数的 16 位编码。
         * @note ARM 路径使用 VCVTB 指令，非 ARM 路径使用 _Float16；不进行数值范围钳位。
         */
        static inline uint16_t Float32ToFloat16(float f);

        /**
         * @brief 将 IEEE 754 半精度位表示转换为单精度浮点数
         * @param h 半精度浮点数的 16 位编码
         * @return 解码后的单精度浮点数，可能为非有限值，由接收逻辑校验。
         */
        static inline float Float16ToFloat32(uint16_t h);

        CanBridge bridge_;                  ///< 本对象持有的 CAN Bridge，底层 CAN 对象由应用管理
        chassis_can_bridge_config config_;  ///< 构造时保存的地址与超时配置

        // 运动指令缓存
        control::chassis_speed command_;       ///< 最近一次有效目标控制量
        bool command_chassis_enable_ = false;  ///< 最近一次有效底盘使能请求
        bool command_system_active_ = false;   ///< 最近一次有效系统活动状态
        uint32_t command_ms_ = 0;              ///< 运动指令的本机接收时间 [ms]
        bool command_seen_ = false;            ///< 是否曾收到有效运动指令，区分未接收与合法的零 tick

        // 功率参数缓存
        control::chassis_power power_;  ///< 最近一次有效功率限制参数
        uint32_t power_ms_ = 0;         ///< 功率参数的本机接收时间 [ms]
        bool power_seen_ = false;       ///< 是否曾收到有效功率参数

        // 实测反馈缓存
        control::chassis_state feedback_;     ///< 最近一次有效实测速度及使能状态
        uint8_t feedback_motors_online_ = 0;  ///< 最近一次有效电机在线位掩码
        uint8_t feedback_status_ = 0;         ///< 最近一次有效状态标志
        uint32_t feedback_ms_ = 0;            ///< 实测反馈的本机接收时间 [ms]
        bool feedback_seen_ = false;          ///< 是否曾收到有效实测反馈

        // 云台与底盘心跳分别缓存
        uint32_t heartbeat_gimbal_seq_ = 0;        ///< 最近一次云台心跳序号
        uint32_t heartbeat_gimbal_uptime_ms_ = 0;  ///< 云台报告的运行时间 [ms]
        uint32_t heartbeat_gimbal_ms_ = 0;         ///< 云台心跳的本机接收时间 [ms]
        bool heartbeat_gimbal_seen_ = false;       ///< 是否曾收到有效云台心跳

        uint32_t heartbeat_chassis_seq_ = 0;        ///< 最近一次底盘心跳序号
        uint32_t heartbeat_chassis_uptime_ms_ = 0;  ///< 底盘报告的运行时间 [ms]
        uint32_t heartbeat_chassis_ms_ = 0;         ///< 底盘心跳的本机接收时间 [ms]
        bool heartbeat_chassis_seen_ = false;       ///< 是否曾收到有效底盘心跳

        uint32_t tx_heartbeat_seq_ = 0;  ///< 下一次发送的心跳序号，两类心跳共用，按 uint32_t 自然回绕
    };
}  // namespace communication
