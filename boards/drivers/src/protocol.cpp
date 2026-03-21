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
#include "dji_remote.h"

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace communication {

    bool Protocol::Receive(package_t package) {
        Heartbeat();
        for (int i = 0; i < package.length; ++i) {
            if (rx_state.mode == rx_state.mode::WAITING_FOR_SOF) {
                // 还未接收到帧头，检测是否为帧头
                if (package.data[i] == SOF) {
                    rx_state.mode = rx_state.mode::RECEIVING_REFEREE;
                    bufferRx[0] = SOF;
                    rx_state.idx = 1;
                } else if (package.data[i] == remote::vt13_packet_t::SOF) {
                    rx_state.mode = rx_state.mode::RECEIVING_VT13;
                    bufferRx[0] = remote::vt13_packet_t::SOF;
                    rx_state.idx = 1;
                }
            } else if (rx_state.mode == rx_state.RECEIVING_REFEREE) {
                // 接收裁判系统的数据
                bufferRx[rx_state.idx++] = package.data[i];

                if (rx_state.idx >= FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN) {
                    int DATA_LENGTH = bufferRx[2] << BYTE | bufferRx[1];
                    if (rx_state.idx >=
                        FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN) {
                        if (VerifyHeader(bufferRx, FRAME_HEADER_LEN) &&
                            VerifyFrame(bufferRx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH +
                                                      FRAME_TAIL_LEN)) {
                            int cmd_id =
                                bufferRx[FRAME_HEADER_LEN + 1] << BYTE | bufferRx[FRAME_HEADER_LEN];
                            ProcessDataRx(cmd_id, bufferRx + FRAME_HEADER_LEN + CMD_ID_LEN,
                                          DATA_LENGTH);
                        }
                        rx_state.mode = rx_state.WAITING_FOR_SOF;
                        rx_state.idx = 0;
                    }
                }
            } else if (rx_state.mode == rx_state.mode::RECEIVING_VT13) {
                // VT13遥控器与图传链路从同一个UART输出，包头不同。
                // Todo: 应该把图传链路和遥控器数据分开处理
                bufferRx[rx_state.idx++] = package.data[i];
                if (rx_state.idx >= static_cast<int>(sizeof(remote::vt13_packet_t))) {
                    if (verify_crc16_check_sum(bufferRx, sizeof(remote::vt13_packet_t))) {
                        ProcessDataRx(REMOTE_CONTROL_VT13, bufferRx, sizeof(remote::vt13_packet_t));
                    }
                    rx_state.mode = rx_state.WAITING_FOR_SOF;
                    rx_state.idx = 0;
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

#ifndef NO_USB

    USBProtocol::USBProtocol(bsp::VirtualUSB* usb, uint32_t txBufferSize = 200,
                             uint32_t rxBufferSize = 200)
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
        usb_protocol_->Receive(communication::package_t{
            usb_protocol_->read_ptr_, static_cast<int>(usb_protocol_->read_len_)});
    }
    USBProtocol::~USBProtocol() {
        delete callback_thread_;
    }
    package_t USBProtocol::Transmit(int cmd_id) {
        package_t package = Protocol::Transmit(cmd_id);
        usb_->Write<false>(package.data, package.length);
        return package;
    }

#endif

    Referee::Referee(bsp::UART* uart) : UARTProtocol(uart) {
        // 设置100ms离线阈值
        SetThreshold(online_threshold_);
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
            case REMOTE_CONTROL_VT13:
                memcpy(&vt13_packet, data, length);
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
                // memset(&target_angle, 0, sizeof(target_angle));
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
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                memcpy(&robot_power_heat_hp_upload, data, length);
                break;
            case GIMBAL_CURRENT_STATUS:
                memcpy(&gimbal_current_status, data, length);
                break;
            case CHASSIS_CURRENT_STATUS:
                memcpy(&chassis_current_status, data, length);
                break;
            case AUTOAIM_ENABLE:
                memcpy(&autoaim_enable, data, length);
                break;
            case ROBOT_STATUS_UPLOAD:
                memcpy(&robot_status_upload, data, length);
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
                memcpy(&pack, data, length);
                break;
            case TARGET_ANGLE:
                // memset(&target_angle, 0, sizeof(target_angle));
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
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                memcpy(&robot_power_heat_hp_upload, data, length);
                break;
            case GIMBAL_CURRENT_STATUS:
                memcpy(&gimbal_current_status, data, length);
                break;
            case CHASSIS_CURRENT_STATUS:
                memcpy(&chassis_current_status, data, length);
                break;
            case AUTOAIM_ENABLE:
                memcpy(&autoaim_enable, data, length);
                break;
            case ROBOT_STATUS_UPLOAD:
                memcpy(&robot_status_upload, data, length);
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
            case GAME_ROBOT_STATUS:
                data_len = sizeof(game_robot_status_t);
                memcpy(data, &game_robot_status, data_len);
                break;
            case GAME_STATUS:
                data_len = sizeof(game_status_t);
                memcpy(data, &game_status, data_len);
                break;
            default:
                data_len = -1;
        }
        return data_len;
    }

#endif

} /* namespace communication */