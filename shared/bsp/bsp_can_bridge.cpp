#include "bsp_can_bridge.h"

#include <cstring>

namespace bsp {

    static void bridge_callback(const uint8_t data[], void* args) {
        CanBridge* bridge = reinterpret_cast<CanBridge*>(args);
        bridge->UpdateData(data);
    }

    CanBridge::CanBridge(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id) {
        can_ = can;
        rx_id_ = rx_id;
        tx_id_ = tx_id;
        can_->RegisterRxCallback(rx_id_, bridge_callback, this);
    }

    void CanBridge::UpdateData(const uint8_t* data) {
        memcpy(&cmd, data, sizeof(bridge_data_t));
        switch (cmd.id) {
            case VX:
                vx = cmd.data_float;
                break;
            case VY:
                vy = cmd.data_float;
                break;
            case RELATIVE_ANGLE:
                relative_angle = cmd.data_float;
                break;
            case START:
                start = cmd.data_bool;
                break;
            case MODE:
                mode = cmd.data_int;
                break;
            case DEAD:
                dead = cmd.data_bool;
                break;
            case SHOOTER_POWER:
                shooter_power = cmd.data_bool;
                break;
            case COOLING_HEAT1:
                cooling_heat1 = cmd.data_float;
                break;
            case COOLING_HEAT2:
                cooling_heat2 = cmd.data_float;
                break;
            case COOLING_LIMIT1:
                cooling_limit1 = cmd.data_float;
                break;
            case COOLING_LIMIT2:
                cooling_limit2 = cmd.data_float;
                break;
            case SPEED_LIMIT1:
                speed_limit1 = cmd.data_float;
                break;
            case SPEED_LIMIT2:
                speed_limit2 = cmd.data_float;
                break;
            default:;
        }
    }

    void CanBridge::TransmitOutput() {
        can_->Transmit(tx_id_, (uint8_t*)&cmd, sizeof(bridge_data_t));
    }

}  // namespace bsp