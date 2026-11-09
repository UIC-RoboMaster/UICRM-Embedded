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

#include "seasons/2027/protocol.h"

#include <string.h>

namespace referee_comm {
    namespace v2027 {

        template <typename T>
        bool Protocol::Assign(const Frame& frame, T& target, size_t expected) {
            if (expected == 0 || expected > sizeof(T) || frame.length != expected)
                return false;
            memcpy(&target, frame.data, expected);
            return true;
        }

        bool Protocol::OnCommand(const Frame& frame) {
            if (frame.data == nullptr)
                return false;

            switch (frame.cmd_id) {
                /* ===== 常规链路 ===== */
                case GAME_STATUS:
                    return Assign(frame, game_status, sizeof(game_status_t));
                case GAME_RESULT:
                    return Assign(frame, game_result, sizeof(game_result_t));
                case GAME_ROBOT_HP:
                    return Assign(frame, game_robot_hp, sizeof(game_robot_HP_t));
                case EVENT_DATA:
                    return Assign(frame, event_data, sizeof(event_data_t));
                case REFEREE_WARNING:
                    return Assign(frame, referee_warning, sizeof(referee_warning_t));
                case DART_INFO:
                    return Assign(frame, dart_info, sizeof(dart_info_t));
                case ROBOT_STATUS:
                    return Assign(frame, robot_status, sizeof(robot_status_t));
                case POWER_HEAT_DATA:
                    return Assign(frame, power_heat_data, sizeof(power_heat_data_t));
                case ROBOT_POS:
                    return Assign(frame, robot_pos, sizeof(robot_pos_t));
                case BUFF:
                    return Assign(frame, buff, sizeof(buff_t));
                case HURT_DATA:
                    return Assign(frame, hurt_data, sizeof(hurt_data_t));
                case SHOOT_DATA:
                    return Assign(frame, shoot_data, sizeof(shoot_data_t));
                case PROJECTILE_ALLOWANCE:
                    return Assign(frame, projectile_allowance, sizeof(projectile_allowance_t));
                case RFID_STATUS:
                    return Assign(frame, rfid_status, sizeof(rfid_status_t));
                case DART_CLIENT_CMD:
                    return Assign(frame, dart_client_cmd, sizeof(dart_client_cmd_t));
                case GROUND_ROBOT_POSITION:
                    return Assign(frame, ground_robot_position, sizeof(ground_robot_position_t));
                case RADAR_MARK_DATA:
                    return Assign(frame, radar_mark_data, sizeof(radar_mark_data_t));
                case SENTRY_INFO:
                    return Assign(frame, sentry_info, sizeof(sentry_info_t));
                case RADAR_INFO:
                    return Assign(frame, radar_info, sizeof(radar_info_t));
                case MAP_PATH:
                    return Assign(frame, map_path, sizeof(map_data_t));

                /* 0x0301 内容数据段长度可变，仅校验头部与上限 */
                case ROBOT_INTERACTION: {
                    constexpr size_t HEADER = 6;
                    if (frame.length < HEADER || frame.length > sizeof(robot_interaction_t))
                        return false;
                    const size_t user_length = frame.length - HEADER;
                    memcpy(&robot_interaction.data_cmd_id, frame.data, 2);
                    memcpy(&robot_interaction.sender_id, frame.data + 2, 2);
                    memcpy(&robot_interaction.receiver_id, frame.data + 4, 2);
                    memcpy(robot_interaction.user_data, frame.data + HEADER, user_length);

                    // 接收方向的子内容再拆一层；图形等发送方向内容保持透传。
                    switch (robot_interaction.data_cmd_id) {
                        case SENTRY_CMD:
                            if (user_length != sizeof(sentry_cmd_t))
                                return false;
                            memcpy(&sentry_cmd, robot_interaction.user_data, sizeof(sentry_cmd_t));
                            break;
                        case RADAR_CMD:
                            if (user_length != sizeof(radar_cmd_t))
                                return false;
                            memcpy(&radar_cmd, robot_interaction.user_data, sizeof(radar_cmd_t));
                            break;
                        default:
                            break;
                    }
                    return true;
                }

                /* ===== 图传链路 ===== */
                case CUSTOM_CONTROL:
                    return Assign(frame, custom_control, sizeof(custom_control_t));

                /* ===== 雷达无线链路 ===== */
                case RADAR_OPPONENT_POS:
                    return Assign(frame, radar_opponent_pos, sizeof(radar_opponent_pos_t));
                case RADAR_OPPONENT_HP:
                    return Assign(frame, radar_opponent_hp, sizeof(radar_opponent_hp_t));
                case RADAR_OPPONENT_ALLOWANCE:
                    return Assign(frame, radar_opponent_allowance, sizeof(radar_opponent_allowance_t));
                case RADAR_OPPONENT_MACRO:
                    return Assign(frame, radar_opponent_macro, sizeof(radar_opponent_macro_t));
                case RADAR_OPPONENT_BUFF:
                    return Assign(frame, radar_opponent_buff, sizeof(radar_opponent_buff_t));
                case RADAR_OPPONENT_KEY:
                    return Assign(frame, radar_opponent_key, sizeof(radar_opponent_key_t));

                /* TODO: 0x0303 MAP_COMMAND / 0x0305 MAP_ROBOT_DATA / 0x0308 CUSTOM_INFO
                 * 已在 protocol.h 定义命令码与结构体，但此处未接线，收到即落入 default 丢弃：
                 *   0x0303 选手端小地图交互（选手端→哨兵）与 0x0305 雷达全场态势（雷达→哨兵）
                 *   仅哨兵/雷达角色会收到，步兵链路上不出现，暂不处理；接入时在上方加 case，
                 *   并在 Protocol 类补充对应状态成员。
                 *   0x0308 为机器人→选手端的发送方向（自定义文本消息），需要时在 OnBuild
                 *   补打包，OnCommand 无需处理。
                 */
                default:
                    /* 未知或已废弃命令：静默忽略，不影响字节流同步 */
                    return false;
            }
        }

        int Protocol::OnBuild(uint16_t cmd_id, uint8_t* out, size_t capacity) {
            switch (cmd_id) {
                /* 内容由 UI 层写入 robot_interaction，此处只负责打包上线 */
                case ROBOT_INTERACTION:
                    if (robot_interaction.data_cmd_id == 0)
                        return -1;
                    if (capacity < sizeof(robot_interaction_t))
                        return -1;
                    memcpy(out, &robot_interaction, sizeof(robot_interaction_t));
                    return static_cast<int>(sizeof(robot_interaction_t));

                case ROBOT_CUSTOM_DATA:
                    if (capacity < sizeof(robot_custom_data_t))
                        return -1;
                    memcpy(out, &robot_custom_data, sizeof(robot_custom_data_t));
                    return static_cast<int>(sizeof(robot_custom_data_t));

                default:
                    return -1;
            }
        }

        bool Protocol::IsInBattle() const {
            // 0x0001 的 game_progress 为 4 表示比赛中。
            return game_status.game_progress == 4;
        }

        bool Protocol::IsDead() const {
            return robot_status.current_HP == 0;
        }

        uint16_t Protocol::BufferEnergy() const {
            return power_heat_data.buffer_energy;
        }

        uint16_t Protocol::BarrelHeat17mm() const {
            return power_heat_data.shooter_17mm_barrel_heat;
        }

    }  // namespace v2027
}  // namespace referee_comm
