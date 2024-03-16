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

#include "protocol.h"

#include <cstring>

#include "crc_check.h"

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace communication {

    bool Protocol::Receive(package_t package) {
        Heartbeat();
        memcpy(bufferRx, package.data, package.length);
        int start_idx;
        int end_idx = 0;
        while (end_idx < package.length) {
            start_idx = end_idx;
            while (++end_idx < package.length && bufferRx[end_idx] != SOF)
                ;
            if (end_idx - start_idx > FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN) {
                int DATA_LENGTH = bufferRx[start_idx + 2] << BYTE | bufferRx[start_idx + 1];
                if (VerifyHeader(bufferRx + start_idx, FRAME_HEADER_LEN) &&
                    VerifyFrame(bufferRx + start_idx,
                                FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN)) {
                    int cmd_id = bufferRx[start_idx + FRAME_HEADER_LEN + 1] << BYTE |
                                 bufferRx[start_idx + FRAME_HEADER_LEN];
                    ProcessDataRx(cmd_id, bufferRx + start_idx + FRAME_HEADER_LEN + CMD_ID_LEN,
                                  DATA_LENGTH);
                }
            }
        }
        return true;
    }

    package_t Protocol::Transmit(int cmd_id) {
        bufferTx[0] = SOF;
        int DATA_LENGTH = ProcessDataTx(cmd_id, bufferTx + FRAME_HEADER_LEN + CMD_ID_LEN);
        if (DATA_LENGTH < 0)
            return package_t{nullptr, 0};
        bufferTx[1] = (uint8_t)((uint32_t)DATA_LENGTH & 0xFF);
        bufferTx[2] = (uint8_t)((uint32_t)DATA_LENGTH >> BYTE);
        bufferTx[3] = (uint32_t)seq++;
        AppendHeader(bufferTx, FRAME_HEADER_LEN);
        bufferTx[5] = (uint8_t)((uint32_t)cmd_id & 0xFF);
        bufferTx[6] = (uint8_t)((uint32_t)cmd_id >> BYTE);
        AppendFrame(bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN);
        return package_t{bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN};
    }

    bool Protocol::VerifyHeader(const uint8_t* data, int length) {
        return verify_crc8_check_sum(data, length);
    }

    bool Protocol::VerifyFrame(const uint8_t* data, int length) {
        return verify_crc16_check_sum(data, length);
    }

    void Protocol::AppendHeader(uint8_t* data, int length) {
        append_crc8_check_sum(data, length);
    }

    void Protocol::AppendFrame(uint8_t* data, int length) {
        append_crc16_check_sum(data, length);
    }

    UARTProtocol::UARTProtocol(bsp::UART* uart) : Protocol() {
        uart_ = uart;
        uart_->SetupRxData(&read_ptr_, &read_len_);
        uart_->RegisterCallback(CallbackWrapper, this);
        bsp::thread_init_t thread_init = {
            .func = callback_thread_func_,
            .args = this,
            .attr = callback_thread_attr_,
        };
        callback_thread_ = new bsp::EventThread(thread_init);
    }
    void UARTProtocol::CallbackWrapper(void* args) {
        UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
        uart_protocol_->callback_thread_->Set();
    }
    void UARTProtocol::callback_thread_func_(void* args) {
        UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
        uart_protocol_->Receive(
            communication::package_t{uart_protocol_->read_ptr_, (int)uart_protocol_->read_len_});
    }
    UARTProtocol::~UARTProtocol() {
        delete callback_thread_;
    }
    package_t UARTProtocol::Transmit(int cmd_id) {
        package_t package = Protocol::Transmit(cmd_id);
        uart_->Write(package.data, package.length);
        return package;
    }

    Referee::Referee(bsp::UART* uart) : UARTProtocol(uart) {
        // 设置100ms离线阈值
        SetThreshold(100);
    }

    bool Referee::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        switch (cmd_id) {
            case GAME_STATUS:
                memcpy(&game_status, data, length);
                break;
            case GAME_RESULT:
                memcpy(&game_result, data, length);
                break;
            case GAME_ROBOT_HP:
                memcpy(&game_robot_HP, data, length);
                break;
            case EVENT_DATA:
                memcpy(&event_data, data, length);
                break;
            case SUPPLY_PROJECTILE_ACTION:
                memcpy(&supply_projectile_action, data, length);
                break;
            case REFEREE_WARNING:
                memcpy(&referee_warning, data, length);
                break;
            case DART_REMAINING_TIME:
                memcpy(&dart_remaining_time, data, length);
                break;
            case GAME_ROBOT_STATUS:
                memcpy(&game_robot_status, data, length);
                break;
            case POWER_HEAT_DATA:
                memcpy(&power_heat_data, data, length);
                break;
            case GAME_ROBOT_POS:
                memcpy(&game_robot_pos, data, length);
                break;
            case BUFF:
                memcpy(&buff, data, length);
                break;
            case AERIAL_ROBOT_ENERGY:
                memcpy(&aerial_robot_energy, data, length);
                break;
            case ROBOT_HURT:
                memcpy(&robot_hurt, data, length);
                break;
            case SHOOT_DATA:
                memcpy(&shoot_data, data, length);
                break;
            case BULLET_REMAINING:
                memcpy(&bullet_remaining, data, length);
                break;
            case RFID_STATUS:
                memcpy(&rfid_status, data, length);
                break;
            case DART_CLIENT_CMD:
                memcpy(&dart_client_cmd, data, length);
                break;
            case REMOTE_CONTROL_DATA:
                memcpy(&remote_control, data, length);
                break;
            default:
                return false;
        }
        return true;
    }

    int Referee::ProcessDataTx(int cmd_id, uint8_t* data) {
        int data_len;
        switch (cmd_id) {
            case STUDENT_INTERACTIVE: {
                switch (graph_content_) {
                    case NO_GRAPH:
                        data_len = -1;
                        break;
                    case DELETE_GRAPH:
                        data_len = sizeof(graphic_delete_t);
                        memcpy(data, &graphic_delete, data_len);
                        break;
                    case SINGLE_GRAPH:
                        data_len = sizeof(graphic_single_t);
                        memcpy(data, &graphic_single, data_len);
                        break;
                    case DOUBLE_GRAPH:
                        data_len = sizeof(graphic_double_t);
                        memcpy(data, &graphic_double, data_len);
                        break;
                    case FIVE_GRAPH:
                        data_len = sizeof(graphic_five_t);
                        memcpy(data, &graphic_five, data_len);
                        break;
                    case SEVEN_GRAPH:
                        data_len = sizeof(graphic_seven_t);
                        memcpy(data, &graphic_seven, data_len);
                        break;
                    case CHAR_GRAPH:
                        data_len = sizeof(graphic_character_t);
                        memcpy(data, &graphic_character, data_len);
                        break;
                    default:
                        data_len = -1;
                }
                graph_content_ = NO_GRAPH;
                break;
            }
            default:
                data_len = -1;
        }
        if (data_len > 0)
            uart_->Write(data, data_len);
        return data_len;
    }

    void Referee::PrepareUIContent(content graph_content) {
        graph_content_ = graph_content;
    }

    Host::Host(bsp::UART* uart) : UARTProtocol(uart) {
    }

    bool Host::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        switch (cmd_id) {
            case PACK:
                memcpy(&pack, data, length);
                break;
            case TARGET_ANGLE:
                memcpy(&target_angle, data, length);
                break;
            case NO_TARGET_FLAG:
                memcpy(&no_target_flag, data, length);
                break;
            case SHOOT_CMD:
                memcpy(&shoot_cmd, data, length);
                break;
            case ROBOT_MOVE_SPEED:
                memcpy(&robot_move, data, length);
                break;
            case ROBOT_STATUS_UPLOAD:
                memcpy(&robot_status_upload, data, length);
                break;
            case ROBOT_POSITION:
                memcpy(&gimbal_current_status, data, length);
                break;
            default:
                return false;
        }
        return true;
    }

    int Host::ProcessDataTx(int cmd_id, uint8_t* data) {
        int data_len;
        switch (cmd_id) {
            case PACK:
                data_len = sizeof(pack_t);
                memcpy(data, &pack, data_len);
                break;
            case TARGET_ANGLE:
                data_len = sizeof(target_angle_t);
                memcpy(data, &target_angle, data_len);
                break;
            case NO_TARGET_FLAG:
                data_len = sizeof(no_target_flag_t);
                memcpy(data, &no_target_flag, data_len);
                break;
            case SHOOT_CMD:
                data_len = sizeof(shoot_cmd_t);
                memcpy(data, &shoot_cmd, data_len);
                break;
            case ROBOT_MOVE_SPEED:
                data_len = sizeof(robot_move_t);
                memcpy(data, &robot_move, data_len);
                break;
            case ROBOT_STATUS_UPLOAD:
                data_len = sizeof(robot_status_upload_t);
                memcpy(data, &robot_status_upload, data_len);
                break;
            case ROBOT_POSITION:
                data_len = sizeof(gimbal_current_status_t);
                memcpy(data, &gimbal_current_status, data_len);
                break;
            default:
                data_len = -1;
        }
        return data_len;
    }

} /* namespace communication */