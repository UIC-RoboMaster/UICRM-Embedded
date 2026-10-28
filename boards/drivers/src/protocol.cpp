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

#include "bsp_usb.h"
#include "crc_check.h"
#include "bsp_os.h"

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace communication {
    void Protocol::DiscardRx(size_t size) {
        rx_used_ -= size;
        std::memmove(bufferRx, bufferRx + size, rx_used_);
    }

    void Protocol::ResyncRx() {
        size_t next = 1;
        while (next < rx_used_ && bufferRx[next] != SOF &&
               bufferRx[next] != controller::RefereeRC::SOF) {
            ++next;
        }
        DiscardRx(next);
    }

    bool Protocol::Receive(package_t package) {
        if (!package.data || package.length <= 0) return false;
        // Preserve the legacy link heartbeat. Input-source health is independent.
        Heartbeat();
        const uint32_t now_ms = bsp::GetHighresTickMilliSec();
        for (int i = 0; i < package.length; ++i) {
            const auto byte = package.data[i];
            if (!rx_used_ && byte != SOF && byte != controller::RefereeRC::SOF) continue;
            // A5 lengths are capped below; every full candidate is consumed or
            // resynchronized before the next append, keeping this buffer bounded.
            bufferRx[rx_used_++] = byte;
            while (rx_used_) {
                if (bufferRx[0] != SOF && bufferRx[0] != controller::RefereeRC::SOF) {
                    ResyncRx();
                    continue;
                }
                if (bufferRx[0] == controller::RefereeRC::SOF) {
                    if (rx_used_ < 2) break;
                    if (bufferRx[1] != controller::RefereeRC::CMD) {
                        ResyncRx();
                        continue;
                    }
                    if (rx_used_ < controller::RefereeRC::FRAME_SIZE) break;
                    // Protocol owns stream framing; the receiver owns VT13
                    // validation, decoding and the independent input timestamp.
                    refereerc_.ResetStream();
                    const auto sequence = refereerc_.GetSequence();
                    refereerc_.UpdateData(bufferRx, controller::RefereeRC::FRAME_SIZE, now_ms);
                    refereerc_.ResetStream();
                    if (refereerc_.GetSequence() == sequence) {
                        ResyncRx();
                        continue;
                    }
                    DiscardRx(controller::RefereeRC::FRAME_SIZE);
                } else {
                    if (rx_used_ < FRAME_HEADER_LEN) break;
                    const int payload = int(bufferRx[1]) | (int(bufferRx[2]) << BYTE);
                    if (!VerifyHeader(bufferRx, FRAME_HEADER_LEN) || payload > MAX_FRAME_LEN - 9) {
                        ResyncRx();
                        continue;
                    }
                    const int total = payload + FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN;
                    if (rx_used_ < static_cast<size_t>(total)) break;
                    if (!VerifyFrame(bufferRx, total)) {
                        ResyncRx();
                        continue;
                    }
                    const int cmd = int(bufferRx[5]) | (int(bufferRx[6]) << BYTE);
                    ProcessDataRx(cmd, bufferRx + FRAME_HEADER_LEN + CMD_ID_LEN, payload);
                    DiscardRx(static_cast<size_t>(total));
                }
            }
        }
        return true;
    }

    package_t Protocol::Transmit(int cmd_id) {
        bufferTx[0] = SOF;
        int DATA_LENGTH = ProcessDataTx(cmd_id, bufferTx + FRAME_HEADER_LEN + CMD_ID_LEN);
        if (DATA_LENGTH < 0 || DATA_LENGTH > MAX_FRAME_LEN - 9)
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
        uart_protocol_->Receive(communication::package_t{uart_protocol_->read_ptr_, (int)uart_protocol_->read_len_});
    }

    UARTProtocol::~UARTProtocol() {
        delete callback_thread_;
    }

    package_t UARTProtocol::Transmit(int cmd_id) {
        package_t package = Protocol::Transmit(cmd_id);
        if (package.data && package.length > 0) uart_->Write(package.data, package.length);
        return package;
    }

#ifndef NO_USB

    USBProtocol::USBProtocol(bsp::VirtualUSB* usb, uint32_t txBufferSize = 200, uint32_t rxBufferSize = 200)
        : Protocol() {
        usb_ = usb;
        usb_->SetupTx(txBufferSize);
        usb_->SetupRx(rxBufferSize);
        usb_->SetupRxData(&read_ptr_, &read_len_);
        usb_->RegisterCallback(CallbackWrapper, this);
        bsp::thread_init_t thread_init = {
            .func = callback_thread_func_,
            .args = this,
            .attr = callback_thread_attr_,
        };
        callback_thread_ = new bsp::EventThread(thread_init);
    }

    void USBProtocol::CallbackWrapper(void* args) {
        USBProtocol* usb_protocol_ = reinterpret_cast<USBProtocol*>(args);
        usb_protocol_->callback_thread_->Set();
    }

    void USBProtocol::callback_thread_func_(void* args) {
        USBProtocol* usb_protocol_ = reinterpret_cast<USBProtocol*>(args);
        usb_protocol_->Receive(
            communication::package_t{usb_protocol_->read_ptr_, static_cast<int>(usb_protocol_->read_len_)}
        );
    }

    USBProtocol::~USBProtocol() {
        delete callback_thread_;
    }

    package_t USBProtocol::Transmit(int cmd_id) {
        package_t package = Protocol::Transmit(cmd_id);
        if (package.data && package.length > 0) usb_->Write<false>(package.data, package.length);
        return package;
    }

#endif

    Referee::Referee(bsp::UART* uart) : UARTProtocol(uart) {
        // 构造阶段初始化判定阈值100 + 150ms
        SetThreshold(online_threshold_);
    }

    bool Referee::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        switch (cmd_id) {
            case GAME_STATUS:
                if (length != static_cast<int>(sizeof(game_status))) return false;
                memcpy(&game_status, data, sizeof(game_status));
                break;
            case GAME_RESULT:
                if (length != static_cast<int>(sizeof(game_result))) return false;
                memcpy(&game_result, data, sizeof(game_result));
                break;
            case GAME_ROBOT_HP:
                if (length != static_cast<int>(sizeof(game_robot_HP))) return false;
                memcpy(&game_robot_HP, data, sizeof(game_robot_HP));
                break;
            case EVENT_DATA:
                if (length != static_cast<int>(sizeof(event_data))) return false;
                memcpy(&event_data, data, sizeof(event_data));
                break;
            case SUPPLY_PROJECTILE_ACTION:
                if (length != static_cast<int>(sizeof(supply_projectile_action))) return false;
                memcpy(&supply_projectile_action, data, sizeof(supply_projectile_action));
                break;
            case REFEREE_WARNING:
                if (length != static_cast<int>(sizeof(referee_warning))) return false;
                memcpy(&referee_warning, data, sizeof(referee_warning));
                break;
            case DART_REMAINING_TIME:
                if (length != static_cast<int>(sizeof(dart_remaining_time))) return false;
                memcpy(&dart_remaining_time, data, sizeof(dart_remaining_time));
                break;
            case GAME_ROBOT_STATUS:
                if (length != static_cast<int>(sizeof(game_robot_status))) return false;
                memcpy(&game_robot_status, data, sizeof(game_robot_status));
                break;
            case POWER_HEAT_DATA:
                if (length != static_cast<int>(sizeof(power_heat_data))) return false;
                memcpy(&power_heat_data, data, sizeof(power_heat_data));
                break;
            case GAME_ROBOT_POS:
                if (length != static_cast<int>(sizeof(game_robot_pos))) return false;
                memcpy(&game_robot_pos, data, sizeof(game_robot_pos));
                break;
            case BUFF:
                if (length != static_cast<int>(sizeof(buff))) return false;
                memcpy(&buff, data, sizeof(buff));
                break;
            case AERIAL_ROBOT_ENERGY:
                if (length != static_cast<int>(sizeof(aerial_robot_energy))) return false;
                memcpy(&aerial_robot_energy, data, sizeof(aerial_robot_energy));
                break;
            case ROBOT_HURT:
                if (length != static_cast<int>(sizeof(robot_hurt))) return false;
                memcpy(&robot_hurt, data, sizeof(robot_hurt));
                break;
            case SHOOT_DATA:
                if (length != static_cast<int>(sizeof(shoot_data))) return false;
                memcpy(&shoot_data, data, sizeof(shoot_data));
                break;
            case BULLET_REMAINING:
                if (length != static_cast<int>(sizeof(bullet_remaining))) return false;
                memcpy(&bullet_remaining, data, sizeof(bullet_remaining));
                break;
            case RFID_STATUS:
                if (length != static_cast<int>(sizeof(rfid_status))) return false;
                memcpy(&rfid_status, data, sizeof(rfid_status));
                break;
            case DART_CLIENT_CMD:
                if (length != static_cast<int>(sizeof(dart_client_cmd))) return false;
                memcpy(&dart_client_cmd, data, sizeof(dart_client_cmd));
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
            case CUSTOM_CLIENT_DATA: {
                data_len = sizeof(custom_client_data_t);
                memcpy(data, &custom_client_data, data_len);
                break;
            }
            default:
                data_len = -1;
        }
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
                if (length != static_cast<int>(sizeof(pack))) return false;
                memcpy(&pack, data, sizeof(pack));
                break;
            case TARGET_ANGLE:
                // memset(&target_angle, 0, sizeof(target_angle));
                if (length != static_cast<int>(sizeof(target_angle))) return false;
                memcpy(&target_angle, data, sizeof(target_angle));
                break;
            case NO_TARGET_FLAG:
                if (length != static_cast<int>(sizeof(no_target_flag))) return false;
                memcpy(&no_target_flag, data, sizeof(no_target_flag));
                break;
            case SHOOT_CMD:
                if (length != static_cast<int>(sizeof(shoot_cmd))) return false;
                memcpy(&shoot_cmd, data, sizeof(shoot_cmd));
                break;
            case ROBOT_MOVE_SPEED:
                if (length != static_cast<int>(sizeof(robot_move))) return false;
                memcpy(&robot_move, data, sizeof(robot_move));
                break;
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                if (length != static_cast<int>(sizeof(robot_power_heat_hp_upload))) return false;
                memcpy(&robot_power_heat_hp_upload, data, sizeof(robot_power_heat_hp_upload));
                break;
            case GIMBAL_CURRENT_STATUS:
                if (length != static_cast<int>(sizeof(gimbal_current_status))) return false;
                memcpy(&gimbal_current_status, data, sizeof(gimbal_current_status));
                break;
            case CHASSIS_CURRENT_STATUS:
                if (length != static_cast<int>(sizeof(chassis_current_status))) return false;
                memcpy(&chassis_current_status, data, sizeof(chassis_current_status));
                break;
            case AUTOAIM_ENABLE:
                if (length != static_cast<int>(sizeof(autoaim_enable))) return false;
                memcpy(&autoaim_enable, data, sizeof(autoaim_enable));
                break;
            case ROBOT_STATUS_UPLOAD:
                if (length != static_cast<int>(sizeof(robot_status_upload))) return false;
                memcpy(&robot_status_upload, data, sizeof(robot_status_upload));
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
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                data_len = sizeof(robot_power_heat_hp_upload_t);
                memcpy(data, &robot_power_heat_hp_upload, data_len);
                break;
            case GIMBAL_CURRENT_STATUS:
                data_len = sizeof(gimbal_current_status_t);
                memcpy(data, &gimbal_current_status, data_len);
                break;
            case CHASSIS_CURRENT_STATUS:
                data_len = sizeof(chassis_current_status_t);
                memcpy(data, &chassis_current_status, data_len);
                break;
            case AUTOAIM_ENABLE:
                data_len = sizeof(autoaim_enable_t);
                memcpy(data, &autoaim_enable, data_len);
                break;
            case ROBOT_STATUS_UPLOAD:
                data_len = sizeof(robot_status_upload_t);
                memcpy(data, &robot_status_upload, data_len);
                break;
            default:
                data_len = -1;
        }
        return data_len;
    }

#ifndef NO_USB

    HostUSB::HostUSB(bsp::VirtualUSB* usb, uint32_t txBufferSize = 200, uint32_t rxBufferSize = 200)
        : USBProtocol(usb, txBufferSize, rxBufferSize) {
    }

    bool HostUSB::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        switch (cmd_id) {
            case PACK:
                if (length != static_cast<int>(sizeof(pack))) return false;
                memcpy(&pack, data, sizeof(pack));
                break;
            case TARGET_ANGLE:
                // memset(&target_angle, 0, sizeof(target_angle));
                if (length != static_cast<int>(sizeof(target_angle))) return false;
                memcpy(&target_angle, data, sizeof(target_angle));
                break;
            case NO_TARGET_FLAG:
                if (length != static_cast<int>(sizeof(no_target_flag))) return false;
                memcpy(&no_target_flag, data, sizeof(no_target_flag));
                break;
            case SHOOT_CMD:
                if (length != static_cast<int>(sizeof(shoot_cmd))) return false;
                memcpy(&shoot_cmd, data, sizeof(shoot_cmd));
                break;
            case ROBOT_MOVE_SPEED:
                if (length != static_cast<int>(sizeof(robot_move))) return false;
                memcpy(&robot_move, data, sizeof(robot_move));
                break;
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                if (length != static_cast<int>(sizeof(robot_power_heat_hp_upload))) return false;
                memcpy(&robot_power_heat_hp_upload, data, sizeof(robot_power_heat_hp_upload));
                break;
            case GIMBAL_CURRENT_STATUS:
                if (length != static_cast<int>(sizeof(gimbal_current_status))) return false;
                memcpy(&gimbal_current_status, data, sizeof(gimbal_current_status));
                break;
            case CHASSIS_CURRENT_STATUS:
                if (length != static_cast<int>(sizeof(chassis_current_status))) return false;
                memcpy(&chassis_current_status, data, sizeof(chassis_current_status));
                break;
            case AUTOAIM_ENABLE:
                if (length != static_cast<int>(sizeof(autoaim_enable))) return false;
                memcpy(&autoaim_enable, data, sizeof(autoaim_enable));
                break;
            case ROBOT_STATUS_UPLOAD:
                if (length != static_cast<int>(sizeof(robot_status_upload))) return false;
                memcpy(&robot_status_upload, data, sizeof(robot_status_upload));
                break;
            default:
                return false;
        }
        return true;
    }

    int HostUSB::ProcessDataTx(int cmd_id, uint8_t* data) {
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
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                data_len = sizeof(robot_power_heat_hp_upload_t);
                memcpy(data, &robot_power_heat_hp_upload, data_len);
                break;
            case GIMBAL_CURRENT_STATUS:
                data_len = sizeof(gimbal_current_status_t);
                memcpy(data, &gimbal_current_status, data_len);
                break;
            case CHASSIS_CURRENT_STATUS:
                data_len = sizeof(chassis_current_status_t);
                memcpy(data, &chassis_current_status, data_len);
                break;
            case AUTOAIM_ENABLE:
                data_len = sizeof(autoaim_enable_t);
                memcpy(data, &autoaim_enable, data_len);
                break;
            case ROBOT_STATUS_UPLOAD:
                data_len = sizeof(robot_status_upload_t);
                memcpy(data, &robot_status_upload, data_len);
                break;
            default:
                data_len = -1;
        }
        return data_len;
    }

#endif
} /* namespace communication */