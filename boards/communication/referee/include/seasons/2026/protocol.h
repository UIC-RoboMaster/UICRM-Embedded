/*###########################################################
# Copyright (c) 2026-2027. BNBU/UIC RoboMaster             #
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

#include <stddef.h>
#include <stdint.h>

#include "referee.h"

/*
 * 2026 赛季载荷定义，依据《RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0》。
 *
 * 官方表 1-5「命令码ID一览」有若干笔误，与详细表冲突，本文档一律以详细表和
 * 官方 C 结构体为准：
 *   0x0203 一览表标 16，详细表(1-14)仅 3 个 float，实际 12
 *   0x0303 一览表标 15，详细表(1-35)为 12
 *   0x0208 一览表标 6，结构体含 projectile_allowance_fortress，实际 8
 *   0x0307 一览表标 103，结构体 delta_x/y 各 49 字节，实际 105
 */

namespace referee_comm {
    namespace v2026 {

        /** @brief 2026 赛季命令码。 */
        enum Cmd : uint16_t {
            /* 常规链路 */
            GAME_STATUS = 0x0001,
            GAME_RESULT = 0x0002,
            GAME_ROBOT_HP = 0x0003,
            EVENT_DATA = 0x0101,
            REFEREE_WARNING = 0x0104,
            DART_INFO = 0x0105,
            ROBOT_STATUS = 0x0201,
            POWER_HEAT_DATA = 0x0202,
            ROBOT_POS = 0x0203,
            BUFF = 0x0204,
            HURT_DATA = 0x0206,
            SHOOT_DATA = 0x0207,
            PROJECTILE_ALLOWANCE = 0x0208,
            RFID_STATUS = 0x0209,
            DART_CLIENT_CMD = 0x020A,
            GROUND_ROBOT_POSITION = 0x020B,
            RADAR_MARK_DATA = 0x020C,
            SENTRY_INFO = 0x020D,
            RADAR_INFO = 0x020E,
            ROBOT_INTERACTION = 0x0301,
            MAP_COMMAND = 0x0303,
            MAP_ROBOT_DATA = 0x0305,
            MAP_PATH = 0x0307,
            CUSTOM_INFO = 0x0308,

            /* 图传链路：自定义控制器 */
            CUSTOM_ROBOT_DATA = 0x0302,
            CUSTOM_CLIENT_DATA = 0x0306,
            ROBOT_CUSTOM_DATA = 0x0309,

            /* 图传链路：自定义客户端 */
            ROBOT_CUSTOM_DATA_2 = 0x0310,
            CUSTOM_CONTROL = 0x0311,

            /* 雷达无线链路 */
            RADAR_OPPONENT_POS = 0x0A01,
            RADAR_OPPONENT_HP = 0x0A02,
            RADAR_OPPONENT_ALLOWANCE = 0x0A03,
            RADAR_OPPONENT_MACRO = 0x0A04,
            RADAR_OPPONENT_BUFF = 0x0A05,
            RADAR_OPPONENT_KEY = 0x0A06,
        };

        /* ===== 0x0001 比赛状态 11 字节 1Hz ===== */
        typedef struct {
            uint8_t game_type : 4;       ///< 1 超级对抗赛，2 单项赛，3 ICRA，4 联盟赛，5 步兵对抗
            uint8_t game_progress : 4;   ///< 0 未开始，1 准备，2 自检，3 倒计时，4 比赛中，5 结算
            uint16_t stage_remain_time;  ///< 当前阶段剩余时间，单位秒
            uint64_t SyncTimeStamp;      ///< NTP 同步的 UNIX 时间
        } __packed game_status_t;
        static_assert(sizeof(game_status_t) == 11, "0x0001 长度应为 11");

        /* ===== 0x0002 比赛结果 1 字节 ===== */
        typedef struct {
            uint8_t winner;  ///< 0 平局，1 红方胜，2 蓝方胜
        } __packed game_result_t;
        static_assert(sizeof(game_result_t) == 1, "0x0002 长度应为 1");

        /* ===== 0x0003 己方机器人血量 16 字节 3Hz ===== */
        typedef struct {
            uint16_t ally_1_robot_HP;  ///< 己方英雄血量
            uint16_t ally_2_robot_HP;  ///< 己方工程血量
            uint16_t ally_3_robot_HP;  ///< 己方 3 号步兵血量
            uint16_t ally_4_robot_HP;  ///< 己方 4 号步兵血量
            uint16_t reserved;         ///< 保留位
            uint16_t ally_7_robot_HP;  ///< 己方哨兵血量
            uint16_t ally_outpost_HP;  ///< 己方前哨站血量
            uint16_t ally_base_HP;     ///< 己方基地血量
        } __packed game_robot_HP_t;
        static_assert(sizeof(game_robot_HP_t) == 16, "0x0003 长度应为 16");

        /* ===== 0x0101 场地事件 4 字节 1Hz ===== */
        typedef struct {
            uint32_t event_data;  ///< 增益点与阵地状态位域
        } __packed event_data_t;
        static_assert(sizeof(event_data_t) == 4, "0x0101 长度应为 4");

        /* ===== 0x0104 裁判警告 3 字节 ===== */
        typedef struct {
            uint8_t level;               ///< 判罚等级
            uint8_t offending_robot_id;  ///< 违规机器人 ID
            uint8_t count;               ///< 违规次数
        } __packed referee_warning_t;
        static_assert(sizeof(referee_warning_t) == 3, "0x0104 长度应为 3");

        /* ===== 0x0105 飞镖发射数据 3 字节 1Hz ===== */
        typedef struct {
            uint8_t dart_remaining_time;  ///< 飞镖发射站剩余时间
            uint16_t dart_info;           ///< 飞镖状态位域
        } __packed dart_info_t;
        static_assert(sizeof(dart_info_t) == 3, "0x0105 长度应为 3");

        /* ===== 0x0201 机器人性能体系 13 字节 10Hz ===== */
        typedef struct {
            uint8_t robot_id;                             ///< 本机器人 ID
            uint8_t robot_level;                          ///< 机器人等级
            uint16_t current_HP;                          ///< 当前血量
            uint16_t maximum_HP;                          ///< 血量上限
            uint16_t shooter_barrel_cooling_value;        ///< 射击热量每秒冷却值
            uint16_t shooter_barrel_heat_limit;           ///< 射击热量上限
            uint16_t chassis_power_limit;                 ///< 底盘功率上限
            uint8_t power_management_gimbal_output : 1;   ///< gimbal 口 24V 输出
            uint8_t power_management_chassis_output : 1;  ///< chassis 口 24V 输出
            uint8_t power_management_shooter_output : 1;  ///< shooter 口 24V 输出
        } __packed robot_status_t;
        static_assert(sizeof(robot_status_t) == 13, "0x0201 长度应为 13");

        /* ===== 0x0202 缓冲能量与射击热量 14 字节 10Hz ===== */
        typedef struct {
            uint16_t reserved_0;                ///< 保留位
            uint16_t reserved_1;                ///< 保留位
            float reserved_2;                   ///< 保留位
            uint16_t buffer_energy;             ///< 缓冲能量，单位 J
            uint16_t shooter_17mm_barrel_heat;  ///< 17mm 射击热量
            uint16_t shooter_42mm_barrel_heat;  ///< 42mm 射击热量
        } __packed power_heat_data_t;
        static_assert(sizeof(power_heat_data_t) == 14, "0x0202 长度应为 14");

        /* ===== 0x0203 机器人位置 12 字节 1Hz ===== */
        typedef struct {
            float x;      ///< x 坐标，单位 m
            float y;      ///< y 坐标，单位 m
            float angle;  ///< 测速模块朝向，单位度，正北为 0
        } __packed robot_pos_t;
        static_assert(sizeof(robot_pos_t) == 12, "0x0203 长度应为 12");

        /* ===== 0x0204 机器人增益与底盘能量 8 字节 3Hz ===== */
        typedef struct {
            uint8_t recovery_buff;       ///< 回血增益百分比
            uint16_t cooling_buff;       ///< 冷却增益直接值
            uint8_t defence_buff;        ///< 防御增益百分比
            uint8_t vulnerability_buff;  ///< 负防御增益百分比
            uint16_t attack_buff;        ///< 攻击增益百分比
            uint8_t remaining_energy;    ///< 剩余能量值反馈位域，初始视为 100%
        } __packed buff_t;
        static_assert(sizeof(buff_t) == 8, "0x0204 长度应为 8");

        /* ===== 0x0206 伤害状态 1 字节 ===== */
        typedef struct {
            uint8_t armor_id : 4;             ///< 装甲或测速模块 ID
            uint8_t HP_deduction_reason : 4;  ///< 掉血类型
        } __packed hurt_data_t;
        static_assert(sizeof(hurt_data_t) == 1, "0x0206 长度应为 1");

        /* ===== 0x0207 实时射击 7 字节 ===== */
        typedef struct {
            uint8_t bullet_type;          ///< 1 为 17mm，2 为 42mm
            uint8_t shooter_number;       ///< 1 为 17mm 发射机构，3 为 42mm 发射机构
            uint8_t launching_frequency;  ///< 弹丸射速，单位 Hz
            float initial_speed;          ///< 弹丸初速度，单位 m/s
        } __packed shoot_data_t;
        static_assert(sizeof(shoot_data_t) == 7, "0x0207 长度应为 7");

        /* ===== 0x0208 允许发弹量 8 字节 10Hz ===== */
        typedef struct {
            uint16_t projectile_allowance_17mm;      ///< 17mm 允许发弹量
            uint16_t projectile_allowance_42mm;      ///< 42mm 允许发弹量
            uint16_t remaining_gold_coin;            ///< 剩余金币
            uint16_t projectile_allowance_fortress;  ///< 堡垒储备 17mm 允许发弹量
        } __packed projectile_allowance_t;
        static_assert(sizeof(projectile_allowance_t) == 8, "0x0208 长度应为 8");

        /* ===== 0x0209 RFID 状态 5 字节 3Hz ===== */
        typedef struct {
            uint32_t rfid_status;   ///< 增益点 RFID 状态位域
            uint8_t rfid_status_2;  ///< 扩展 RFID 状态位域
        } __packed rfid_status_t;
        static_assert(sizeof(rfid_status_t) == 5, "0x0209 长度应为 5");

        /* ===== 0x020A 飞镖选手端指令 6 字节 3Hz ===== */
        typedef struct {
            uint8_t dart_launch_opening_status;  ///< 1 关闭，2 开关中，0 已开启
            uint8_t reserved;                    ///< 保留位
            uint16_t target_change_time;         ///< 切换目标时的剩余时间，单位秒
            uint16_t latest_launch_cmd_time;     ///< 最后确认发射指令时的剩余时间，单位秒
        } __packed dart_client_cmd_t;
        static_assert(sizeof(dart_client_cmd_t) == 6, "0x020A 长度应为 6");

        /* ===== 0x020B 地面机器人位置 40 字节 1Hz ===== */
        typedef struct {
            float hero_x;        ///< 己方英雄 x，单位 m
            float hero_y;        ///< 己方英雄 y，单位 m
            float engineer_x;    ///< 己方工程 x，单位 m
            float engineer_y;    ///< 己方工程 y，单位 m
            float standard_3_x;  ///< 己方 3 号步兵 x，单位 m
            float standard_3_y;  ///< 己方 3 号步兵 y，单位 m
            float standard_4_x;  ///< 己方 4 号步兵 x，单位 m
            float standard_4_y;  ///< 己方 4 号步兵 y，单位 m
            float reserved_0;    ///< 保留位
            float reserved_1;    ///< 保留位
        } __packed ground_robot_position_t;
        static_assert(sizeof(ground_robot_position_t) == 40, "0x020B 长度应为 40");

        /* ===== 0x020C 雷达标记进度 2 字节 1Hz ===== */
        typedef struct {
            uint16_t mark_progress;  ///< 双方易伤与特殊标识位域
        } __packed radar_mark_data_t;
        static_assert(sizeof(radar_mark_data_t) == 2, "0x020C 长度应为 2");

        /* ===== 0x020D 哨兵自主决策信息 6 字节 1Hz ===== */
        typedef struct {
            uint32_t sentry_info;    ///< 兑换与复活状态位域
            uint16_t sentry_info_2;  ///< 姿态与脱战状态位域
        } __packed sentry_info_t;
        static_assert(sizeof(sentry_info_t) == 6, "0x020D 长度应为 6");

        /* ===== 0x020E 雷达自主决策信息 1 字节 1Hz ===== */
        typedef struct {
            uint8_t radar_info;  ///< 双倍易伤与加密等级位域
        } __packed radar_info_t;
        static_assert(sizeof(radar_info_t) == 1, "0x020E 长度应为 1");

        /* ===== 0x0301 机器人交互数据 118 字节 ===== */

        /** @brief 交互数据子内容 ID。 */
        enum InteractionCmd : uint16_t {
            LAYER_DELETE = 0x0100,
            FIGURE_ONE = 0x0101,
            FIGURE_TWO = 0x0102,
            FIGURE_FIVE = 0x0103,
            FIGURE_SEVEN = 0x0104,
            FIGURE_CHARACTER = 0x0110,
            SENTRY_CMD = 0x0120,
            RADAR_CMD = 0x0121,
        };

        typedef struct {
            uint16_t data_cmd_id;    ///< 子内容 ID
            uint16_t sender_id;      ///< 发送者 ID
            uint16_t receiver_id;    ///< 接收者 ID
            uint8_t user_data[112];  ///< 内容数据段，最大 112 字节
        } __packed robot_interaction_t;
        static_assert(sizeof(robot_interaction_t) == 118, "0x0301 长度应为 118");

        /** @brief 删除图层 2 字节。 */
        typedef struct {
            uint8_t delete_type;  ///< 0 空操作，1 删除图层，2 删除所有
            uint8_t layer;        ///< 图层数 0~9
        } __packed interaction_layer_delete_t;
        static_assert(sizeof(interaction_layer_delete_t) == 2, "0x0100 长度应为 2");

        /** @brief 单个图形 15 字节。 */
        typedef struct {
            uint8_t figure_name[3];     ///< 图形名，作为删除与修改的索引
            uint32_t operate_type : 3;  ///< 0 空操作，1 增加，2 修改，3 删除
            uint32_t figure_type : 3;  ///< 0 直线，1 矩形，2 正圆，3 椭圆，4 圆弧，5 浮点，6 整型，7 字符
            uint32_t layer : 4;       ///< 图层数 0~9
            uint32_t color : 4;       ///< 颜色
            uint32_t details_a : 9;   ///< 半径或字体大小等
            uint32_t details_b : 9;   ///< 线宽或字符长度等
            uint32_t width : 10;      ///< 线宽
            uint32_t start_x : 11;    ///< 起点 x
            uint32_t start_y : 11;    ///< 起点 y
            uint32_t details_c : 10;  ///< 数字或字符内容
            uint32_t details_d : 11;  ///< 起始角度或整数部分
            uint32_t details_e : 11;  ///< 结束角度或小数部分
        } __packed interaction_figure_t;
        static_assert(sizeof(interaction_figure_t) == 15, "单个图形长度应为 15");

        /** @brief 0x0110 字符图形 45 字节，机器人发给选手端。 */
        typedef struct {
            interaction_figure_t graphic;  ///< 字符配置，格式与单图形相同
            uint8_t data[30];              ///< utf-16 编码字符
        } __packed ext_client_custom_character_t;
        static_assert(sizeof(ext_client_custom_character_t) == 45, "0x0110 长度应为 45");

        /** @brief 0x0120 哨兵自主决策指令 4 字节，选手端发给哨兵。 */
        typedef struct {
            uint32_t sentry_cmd;  ///< bit0 确认复活，bit1 兑换立即复活，bit2-21 兑换发弹量
        } __packed sentry_cmd_t;
        static_assert(sizeof(sentry_cmd_t) == 4, "0x0120 长度应为 4");

        /** @brief 0x0121 雷达自主决策指令 13 字节，选手端发给雷达。 */
        typedef struct {
            uint8_t radar_cmd;          ///< 请求触发双倍易伤的次数
            uint8_t password_cmd;       ///< 是否请求修改密钥
            uint8_t password_1;         ///< 密钥第 1 个字符
            uint8_t password_2;         ///< 密钥第 2 个字符
            uint8_t password_3;         ///< 密钥第 3 个字符
            uint8_t password_4;         ///< 密钥第 4 个字符
            uint8_t password_5;         ///< 密钥第 5 个字符
            uint8_t password_6;         ///< 密钥第 6 个字符
            uint8_t armor_fortify_cmd;  ///< 是否请求触发装甲加固
            uint16_t fortify_center_x;  ///< 加固区域圆心 x，单位 mm
            uint16_t fortify_center_y;  ///< 加固区域圆心 y，单位 mm
        } __packed radar_cmd_t;
        static_assert(sizeof(radar_cmd_t) == 13, "0x0121 长度应为 13");

        /* ===== 0x0303 选手端小地图交互 12 字节 ===== */
        typedef struct {
            float target_position_x;  ///< 目标 x，单位 m
            float target_position_y;  ///< 目标 y，单位 m
            uint8_t cmd_keyboard;     ///< 键盘按键状态
            uint8_t target_robot_id;  ///< 目标机器人 ID
            uint16_t cmd_source;      ///< 信息来源机器人 ID
        } __packed map_command_t;
        static_assert(sizeof(map_command_t) == 12, "0x0303 长度应为 12");

        /* ===== 0x0305 小地图机器人数据 48 字节 ===== */
        typedef struct {
            uint16_t opponent_hero_position_x;        ///< 对方英雄 x，单位 cm
            uint16_t opponent_hero_position_y;        ///< 对方英雄 y，单位 cm
            uint16_t opponent_engineer_position_x;    ///< 对方工程 x，单位 cm
            uint16_t opponent_engineer_position_y;    ///< 对方工程 y，单位 cm
            uint16_t opponent_infantry_3_position_x;  ///< 对方 3 号步兵 x，单位 cm
            uint16_t opponent_infantry_3_position_y;  ///< 对方 3 号步兵 y，单位 cm
            uint16_t opponent_infantry_4_position_x;  ///< 对方 4 号步兵 x，单位 cm
            uint16_t opponent_infantry_4_position_y;  ///< 对方 4 号步兵 y，单位 cm
            uint16_t opponent_aerial_position_x;      ///< 对方空中机器人 x，单位 cm
            uint16_t opponent_aerial_position_y;      ///< 对方空中机器人 y，单位 cm
            uint16_t opponent_sentry_position_x;      ///< 对方哨兵 x，单位 cm
            uint16_t opponent_sentry_position_y;      ///< 对方哨兵 y，单位 cm
            uint16_t ally_hero_position_x;            ///< 己方英雄 x，单位 cm
            uint16_t ally_hero_position_y;            ///< 己方英雄 y，单位 cm
            uint16_t ally_engineer_position_x;        ///< 己方工程 x，单位 cm
            uint16_t ally_engineer_position_y;        ///< 己方工程 y，单位 cm
            uint16_t ally_infantry_3_position_x;      ///< 己方 3 号步兵 x，单位 cm
            uint16_t ally_infantry_3_position_y;      ///< 己方 3 号步兵 y，单位 cm
            uint16_t ally_infantry_4_position_x;      ///< 己方 4 号步兵 x，单位 cm
            uint16_t ally_infantry_4_position_y;      ///< 己方 4 号步兵 y，单位 cm
            uint16_t ally_aerial_position_x;          ///< 己方空中机器人 x，单位 cm
            uint16_t ally_aerial_position_y;          ///< 己方空中机器人 y，单位 cm
            uint16_t ally_sentry_position_x;          ///< 己方哨兵 x，单位 cm
            uint16_t ally_sentry_position_y;          ///< 己方哨兵 y，单位 cm
        } __packed map_robot_data_t;
        static_assert(sizeof(map_robot_data_t) == 48, "0x0305 长度应为 48");

        /* ===== 0x0307 小地图路径 105 字节 ===== */
        typedef struct {
            uint8_t intention;          ///< 意图
            uint16_t start_position_x;  ///< 路径起点 x，单位 dm
            uint16_t start_position_y;  ///< 路径起点 y，单位 dm
            int8_t delta_x[49];         ///< 路径点 x 增量数组，单位 dm
            int8_t delta_y[49];         ///< 路径点 y 增量数组，单位 dm
            uint16_t sender_id;         ///< 发送者 ID
        } __packed map_data_t;
        static_assert(sizeof(map_data_t) == 105, "0x0307 长度应为 105");

        /* ===== 0x0308 选手端自定义消息 34 字节 ===== */
        typedef struct {
            uint16_t sender_id;     ///< 发送者 ID
            uint16_t receiver_id;   ///< 接收者 ID
            uint8_t user_data[30];  ///< utf-16 编码字符
        } __packed custom_info_t;
        static_assert(sizeof(custom_info_t) == 34, "0x0308 长度应为 34");

        /* ===== 0x0302 自定义控制器发给机器人 30 字节 ===== */
        typedef struct {
            uint8_t data[30];  ///< 自定义数据
        } __packed custom_robot_data_t;
        static_assert(sizeof(custom_robot_data_t) == 30, "0x0302 长度应为 30");

        /* ===== 0x0309 机器人发给自定义控制器 30 字节 ===== */
        typedef struct {
            uint8_t data[30];  ///< 自定义数据
        } __packed robot_custom_data_t;
        static_assert(sizeof(robot_custom_data_t) == 30, "0x0309 长度应为 30");

        /* ===== 0x0306 自定义控制器与选手端交互 8 字节 ===== */
        typedef struct {
            uint16_t key_value;        ///< 键盘按键状态
            uint16_t x_position : 12;  ///< 鼠标 x 位置
            uint16_t mouse_left : 4;   ///< 鼠标左键状态
            uint16_t y_position : 12;  ///< 鼠标 y 位置
            uint16_t mouse_right : 4;  ///< 鼠标右键状态
            uint16_t reserved;         ///< 保留位
        } __packed custom_client_data_t;
        static_assert(sizeof(custom_client_data_t) == 8, "0x0306 长度应为 8");

        /* ===== 0x0310 机器人发给自定义客户端 300 字节 ===== */
        typedef struct {
            uint8_t data[300];  ///< 自定义数据，无重传机制
        } __packed robot_custom_data_2_t;
        static_assert(sizeof(robot_custom_data_2_t) == 300, "0x0310 长度应为 300");

        /* ===== 0x0311 自定义客户端发给机器人 30 字节 ===== */
        typedef struct {
            uint8_t data[30];  ///< 自定义指令
        } __packed custom_control_t;
        static_assert(sizeof(custom_control_t) == 30, "0x0311 长度应为 30");

        /* ===== 0x0A01 对方机器人位置 24 字节 ===== */
        typedef struct {
            uint16_t opponent_hero_x;        ///< 对方英雄 x，单位 cm
            uint16_t opponent_hero_y;        ///< 对方英雄 y，单位 cm
            uint16_t opponent_engineer_x;    ///< 对方工程 x，单位 cm
            uint16_t opponent_engineer_y;    ///< 对方工程 y，单位 cm
            uint16_t opponent_infantry_3_x;  ///< 对方 3 号步兵 x，单位 cm
            uint16_t opponent_infantry_3_y;  ///< 对方 3 号步兵 y，单位 cm
            uint16_t opponent_infantry_4_x;  ///< 对方 4 号步兵 x，单位 cm
            uint16_t opponent_infantry_4_y;  ///< 对方 4 号步兵 y，单位 cm
            uint16_t opponent_aerial_x;      ///< 对方空中机器人 x，单位 cm
            uint16_t opponent_aerial_y;      ///< 对方空中机器人 y，单位 cm
            uint16_t opponent_sentry_x;      ///< 对方哨兵 x，单位 cm
            uint16_t opponent_sentry_y;      ///< 对方哨兵 y，单位 cm
        } __packed radar_opponent_pos_t;
        static_assert(sizeof(radar_opponent_pos_t) == 24, "0x0A01 长度应为 24");

        /* ===== 0x0A02 对方机器人血量 12 字节 ===== */
        typedef struct {
            uint16_t opponent_1_robot_HP;  ///< 对方 1 号英雄血量
            uint16_t opponent_2_robot_HP;  ///< 对方 2 号工程血量
            uint16_t opponent_3_robot_HP;  ///< 对方 3 号步兵血量
            uint16_t opponent_4_robot_HP;  ///< 对方 4 号步兵血量
            uint16_t reserved;             ///< 保留位
            uint16_t opponent_7_robot_HP;  ///< 对方 7 号哨兵血量
        } __packed radar_opponent_hp_t;
        static_assert(sizeof(radar_opponent_hp_t) == 12, "0x0A02 长度应为 12");

        /* ===== 0x0A03 对方允许发弹量 10 字节 ===== */
        typedef struct {
            uint16_t opponent_1_allowance;  ///< 对方 1 号英雄允许发弹量
            uint16_t opponent_3_allowance;  ///< 对方 3 号步兵允许发弹量
            uint16_t opponent_4_allowance;  ///< 对方 4 号步兵允许发弹量
            uint16_t reserved;              ///< 保留位
            uint16_t opponent_7_allowance;  ///< 对方 7 号哨兵允许发弹量
        } __packed radar_opponent_allowance_t;
        static_assert(sizeof(radar_opponent_allowance_t) == 10, "0x0A03 长度应为 10");

        /* ===== 0x0A04 对方宏观状态 8 字节 ===== */
        typedef struct {
            uint16_t opponent_remaining_coin;  ///< 对方剩余金币数
            uint16_t opponent_total_coin;      ///< 对方累计总金币数
            uint32_t opponent_status;          ///< 占领状态位域，2026 为 32 位
        } __packed radar_opponent_macro_t;
        static_assert(sizeof(radar_opponent_macro_t) == 8, "0x0A04 长度应为 8");

        /** @brief 单个机器人的增益，7 字节。 */
        typedef struct {
            uint8_t recovery_buff;       ///< 回血增益百分比
            uint16_t cooling_buff;       ///< 冷却增益直接值
            uint8_t defence_buff;        ///< 防御增益百分比
            uint8_t vulnerability_buff;  ///< 负防御增益百分比
            uint16_t attack_buff;        ///< 攻击增益百分比
        } __packed opponent_buff_t;
        static_assert(sizeof(opponent_buff_t) == 7, "单个增益长度应为 7");

        /* ===== 0x0A05 对方各机器人增益 36 字节 ===== */
        typedef struct {
            opponent_buff_t hero;        ///< 对方英雄增益
            opponent_buff_t engineer;    ///< 对方工程增益
            opponent_buff_t infantry_3;  ///< 对方 3 号步兵增益
            opponent_buff_t infantry_4;  ///< 对方 4 号步兵增益
            opponent_buff_t sentry;      ///< 对方哨兵增益
            uint8_t sentry_posture;      ///< 对方哨兵姿态 1 进攻，2 防御，3 移动
        } __packed radar_opponent_buff_t;
        static_assert(sizeof(radar_opponent_buff_t) == 36, "0x0A05 长度应为 36");

        /* ===== 0x0A06 对方干扰波密钥 6 字节 ===== */
        typedef struct {
            uint8_t key[6];  ///< 6 个 ASCII 字母或数字
        } __packed radar_opponent_key_t;
        static_assert(sizeof(radar_opponent_key_t) == 6, "0x0A06 长度应为 6");

        /**
         * @brief 2026 赛季解包实现。
         * @details 每条命令按官方详细表的数据段长度严格校验，长度不符即丢弃该帧；
         * 未知命令静默忽略，保证与其它赛季固件共存于同一总线时互不干扰。
         * 并发：OnCommand 在接收线程中调用，跨线程读取状态须由调用方保证一致性。
         */
        class Protocol final : public ::referee_comm::Protocol {
          public:
            bool OnCommand(const Frame& frame) override;
            int OnBuild(uint16_t cmd_id, uint8_t* out, size_t capacity) override;

            /* 常规链路状态，未收到的命令保持零值 */
            game_status_t game_status{};
            game_result_t game_result{};
            game_robot_HP_t game_robot_hp{};
            event_data_t event_data{};
            referee_warning_t referee_warning{};
            dart_info_t dart_info{};
            robot_status_t robot_status{};
            power_heat_data_t power_heat_data{};
            robot_pos_t robot_pos{};
            buff_t buff{};
            hurt_data_t hurt_data{};
            shoot_data_t shoot_data{};
            projectile_allowance_t projectile_allowance{};
            rfid_status_t rfid_status{};
            dart_client_cmd_t dart_client_cmd{};
            ground_robot_position_t ground_robot_position{};
            radar_mark_data_t radar_mark_data{};
            sentry_info_t sentry_info{};
            radar_info_t radar_info{};
            map_data_t map_path{};

            /* 0x0301 机器人交互数据，接收方向按 data_cmd_id 分派 */
            robot_interaction_t robot_interaction{};
            sentry_cmd_t sentry_cmd{};  ///< 子内容 0x0120，选手端下发的哨兵指令
            radar_cmd_t radar_cmd{};    ///< 子内容 0x0121，选手端下发的雷达指令

            /* 图传链路 */
            custom_robot_data_t custom_robot_data{};      ///< 0x0302 接收
            robot_custom_data_t robot_custom_data{};      ///< 0x0309 待发送
            robot_custom_data_2_t robot_custom_data_2{};  ///< 0x0310 待发送
            custom_client_data_t custom_client_data{};    ///< 0x0306 发送
            custom_control_t custom_control{};            ///< 0x0311 接收

            /* 雷达无线链路 */
            radar_opponent_pos_t radar_opponent_pos{};
            radar_opponent_hp_t radar_opponent_hp{};
            radar_opponent_allowance_t radar_opponent_allowance{};
            radar_opponent_macro_t radar_opponent_macro{};
            radar_opponent_buff_t radar_opponent_buff{};
            radar_opponent_key_t radar_opponent_key{};

            /** @brief 比赛是否已进入进行中阶段。 */
            bool IsInBattle() const;
            /** @brief 本机器人是否已阵亡。 */
            bool IsDead() const;
            /** @brief 底盘剩余缓冲能量，单位 J。 */
            uint16_t BufferEnergy() const;
            /** @brief 17mm 枪口当前热量。 */
            uint16_t BarrelHeat17mm() const;
            /** @brief 17mm 剩余允许发弹量。 */
            uint16_t BulletAllowance17mm() const;

          private:
            /**
             * @brief 校验长度并整体拷贝到目标状态。
             * @param[in] frame 完整帧视图。
             * @param[out] target 目标状态。
             * @param[in] expected 官方数据段长度。
             * @return 长度匹配并已拷贝时为 true。
             */
            template <typename T>
            bool Assign(const Frame& frame, T& target, size_t expected);
        };

    }  // namespace v2026
}  // namespace referee_comm
