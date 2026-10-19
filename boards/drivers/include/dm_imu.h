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

#pragma once

#include "bsp_can.h"
#include "connection_driver.h"

namespace imu {

    // 寄存器编号沿用厂商 MC02 CAN 例程。
    enum class DMIMURegister : uint8_t {
        REBOOT = 0,              // 重启 IMU 模块
        ACCEL = 1,               // 加速度数据
        GYRO = 2,                // 角速度数据（陀螺仪）
        EULER = 3,               // 欧拉角数据
        QUATERNION = 4,          // 四元数数据
        SET_ZERO = 5,            // 姿态置零
        ACCEL_CALIBRATION = 6,   // 加速度计校准
        GYRO_CALIBRATION = 7,    // 陀螺仪校准
        MAG_CALIBRATION = 8,     // 磁力计校准
        COMMUNICATION_PORT = 9,  // 通信接口选择
        ACTIVE_DELAY = 10,       // 主动上报间隔，单位 ms
        ACTIVE_MODE = 11,        // 上报模式：0 为请求模式，1 为主动模式
        CAN_BAUDRATE = 12,       // CAN 波特率配置
        CAN_ID = 13,             // 模块从机 ID，发送命令时使用的目标 CAN ID
        MASTER_ID = 14,          // 反馈主机 ID，接收模块数据时使用的 CAN ID
        OUTPUT_SELECTION = 15,   // 选择主动上报的数据类型
        SAVE_PARAMETERS = 254,   // 保存参数，掉电后保留
        RESTORE_SETTINGS = 255,  // 恢复默认设置
    };

    struct DMIMU_init_t {
        bsp::CAN* can = nullptr;
        uint16_t tx_id = 0x01;
        uint16_t rx_id = 0x11;
        uint32_t online_threshold = 100;  // Milliseconds.
    };

    struct DMIMU_data_t {
        float INS_quat[4] = {};   // 四元数 W / X / Y / Z.
        float INS_angle[3] = {};  // 姿态角 Yaw / pitch / roll, rad.
        float INS_accel[3] = {};  // 加速度 X / Y / Z, m/s^2.
        float INS_gyro[3] = {};   // 角速度 X / Y / Z, rad/s.
        float Temp = 0.0f;        // Degrees Celsius, received with acceleration.
        uint8_t received_mask = 0;
        uint32_t update_count = 0;
    };

    /**
     * @brief 通过项目共享的 CAN 管理对象与 DM-IMU 通信。
     * @note CAN 管理对象应先于驱动创建，两个对象均应在固件运行期间保持有效。
     *       同一总线上的各模块必须使用不同的反馈 ID；构造函数不会配置模块。
     */
    class DMIMU : public driver::ConnectionDriver {
      public:
        static constexpr uint8_t ACCEL_RECEIVED = 1 << 0;
        static constexpr uint8_t GYRO_RECEIVED = 1 << 1;
        static constexpr uint8_t EULER_RECEIVED = 1 << 2;
        static constexpr uint8_t QUATERNION_RECEIVED = 1 << 3;
        static constexpr uint8_t ALL_RECEIVED = 0x0f;

        /**
         * @brief 使用初始化结构体创建驱动并注册 CAN 接收回调。
         * @param init CAN 管理对象、命令 ID、反馈 ID 和离线判定时间。
         * @note 只完成本地初始化，不会修改模块配置或开始周期请求。
         */
        explicit DMIMU(DMIMU_init_t init);

        /**
         * @brief 直接指定通信参数创建驱动并注册 CAN 接收回调。
         * @param can 已创建的 CAN 管理对象；共享总线时复用已有对象。
         * @param tx_id 发送命令使用的模块从机 ID，默认 0x01，范围 0x000～0x7ff。
         * @param rx_id 接收反馈使用的主机 ID，默认 0x11，范围 0x000～0x7ff。
         * @param online_threshold 根据有效测量帧判定离线的时间，单位 ms，默认 100。
         * @note can 必须有效；这些参数不会修改模块自身的 ID 或上报模式。
         */
        explicit DMIMU(bsp::CAN* can, uint16_t tx_id = 0x01, uint16_t rx_id = 0x11, uint32_t online_threshold = 100);

        /**
         * @brief 禁止复制构造驱动对象。
         * @note CAN 接收回调绑定当前对象地址，不能通过复制对象转移绑定。
         */
        DMIMU(const DMIMU&) = delete;

        /**
         * @brief 禁止复制赋值，避免覆盖已注册回调的驱动对象。
         */
        DMIMU& operator=(const DMIMU&) = delete;

        /**
         * @brief 禁止移动构造，保持 CAN 接收回调绑定的对象地址有效。
         */
        DMIMU(DMIMU&&) = delete;

        /**
         * @brief 禁止移动赋值，避免破坏已有 CAN 接收回调绑定。
         */
        DMIMU& operator=(DMIMU&&) = delete;

        /**
         * @brief 向模块发送读取指定寄存器的命令。
         * @param reg 要读取的寄存器编号。
         * @retval true 命令已加入 CAN 发送队列。
         * @retval false 命令未能加入 CAN 发送队列。
         * @note 不等待回复，也不返回寄存器数值；当前驱动只解析四类测量反馈。
         */
        bool ReadReg(DMIMURegister reg);

        /**
         * @brief 向模块发送写入指定寄存器的命令。
         * @param reg 要写入的寄存器编号。
         * @param value 写入的 32 位参数，具体含义由寄存器定义。
         * @retval true 命令已加入 CAN 发送队列。
         * @retval false 命令未能加入 CAN 发送队列。
         * @note 不等待模块确认，返回 true 不代表参数已经生效或保存。
         */
        bool WriteReg(DMIMURegister reg, uint32_t value);

        /**
         * @brief 向模块请求加速度和温度数据。
         * @retval true 请求已加入 CAN 发送队列。
         * @retval false 请求未能加入 CAN 发送队列。
         * @note 只发送请求；回复到达后由 CAN 回调更新 INS_accel（m/s²，包含重力）和 Temp（℃）。
         */
        bool getAccel();

        /**
         * @brief 向模块请求陀螺仪角速度数据。
         * @retval true 请求已加入 CAN 发送队列。
         * @retval false 请求未能加入 CAN 发送队列。
         * @note 只发送请求；回复到达后由 CAN 回调更新 INS_gyro，顺序为 X/Y/Z，单位 rad/s。
         */
        bool getGyro();

        /**
         * @brief 向模块请求欧拉角数据。
         * @retval true 请求已加入 CAN 发送队列。
         * @retval false 请求未能加入 CAN 发送队列。
         * @note 只发送请求；回复到达后由 CAN 回调更新 INS_angle，顺序为 yaw/pitch/roll，单位 rad。
         */
        bool getEuler();

        /**
         * @brief 向模块请求姿态四元数数据。
         * @retval true 请求已加入 CAN 发送队列。
         * @retval false 请求未能加入 CAN 发送队列。
         * @note 只发送请求；回复到达后由 CAN 回调更新 INS_quat，顺序为 W/X/Y/Z。
         */
        bool getQuaternion();

        /**
         * @brief 设置模块的测量数据上报模式。
         * @param active true 为主动上报，false 为收到请求后回复。
         * @retval true 设置命令已加入 CAN 发送队列。
         * @retval false 设置命令未能加入 CAN 发送队列。
         * @note 不等待模块确认、不自动保存；主动上报的数据类型取决于模块输出选择配置。
         */
        bool SetActiveMode(bool active);

        /**
         * @brief 设置模块主动上报的时间间隔。
         * @param delay_ms 主动上报间隔，单位 ms，取值范围为 1～1000。
         * @retval true 设置命令已加入 CAN 发送队列。
         * @retval false 间隔超出范围，或命令未能加入 CAN 发送队列。
         * @note 不等待模块确认，不会自动切换到主动模式或修改 C 板任务的请求周期。
         */
        bool SetActiveDelay(uint32_t delay_ms);

        /**
         * @brief 请求模块保存当前参数，使配置在掉电后保留。
         * @retval true 保存命令已加入 CAN 发送队列。
         * @retval false 保存命令未能加入 CAN 发送队列。
         * @note 不等待模块确认，返回 true 不代表保存已经完成；需要持久化配置时再调用。
         */
        bool SaveParameters();

        /**
         * @brief 复制当前测量数据和接收状态，避免复制过程中被中断更新。
         * @return 使用 C 板成员名称的测量数据快照，欧拉角单位为 rad。
         * @note 短暂屏蔽中断后恢复原中断状态，不会发送请求。
         *       received_mask 只记录启动以来收到过的类型，不表示数据仍在更新。
         *       各类数据来自不同 CAN 帧，快照不代表同时测量；通信在线状态使用 IsOnline() 判断。
         */
        DMIMU_data_t GetData() const;

        volatile float INS_quat[4] = {};   // 四元数 W / X / Y / Z.
        volatile float INS_angle[3] = {};  // 姿态角 Yaw / pitch / roll, rad.
        volatile float INS_accel[3] = {};  // 加速度 X / Y / Z, m/s^2.
        volatile float INS_gyro[3] = {};   // 角速度 X / Y / Z, rad/s.
        volatile float Temp = 0.0f;        // Degrees Celsius.

      private:
        /**
         * @brief 将 CAN 接收回调转发给对应的驱动实例。
         * @param[in] data 模块反馈的 8 字节数据。
         * @param[in] args 注册回调时绑定的 DMIMU 对象指针。
         * @note 在 CAN 接收中断中调用。
         */
        static void CallbackWrapper(const uint8_t data[], void* args);

        /**
         * @brief 解析测量反馈，更新数据成员、接收标志、帧计数和在线心跳。
         * @param[in] data 模块反馈的 8 字节数据，首字节为测量类型。
         * @note 只处理加速度、角速度、欧拉角和四元数；配置回复及未知类型会被忽略。
         */
        void UpdateData(const uint8_t data[]);

        /**
         * @brief 按模块协议打包并发送一条 8 字节标准 CAN 命令。
         * @param reg 目标寄存器编号。
         * @param write true 表示写命令，false 表示读命令。
         * @param value 以小端序发送的 32 位参数，读命令传入 0。
         * @retval true 命令已加入 CAN 发送队列。
         * @retval false 命令未能加入 CAN 发送队列。
         */
        bool SendCommand(DMIMURegister reg, bool write, uint32_t value);

        bsp::CAN* can_;
        uint16_t tx_id_;
        volatile uint8_t received_mask_ = 0;
        volatile uint32_t update_count_ = 0;
    };

}  // namespace imu
