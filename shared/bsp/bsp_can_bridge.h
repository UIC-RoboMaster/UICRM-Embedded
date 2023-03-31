#pragma once

#include "bsp_can.h"

namespace bsp {

    typedef enum {
        VX,
        VY,
        RELATIVE_ANGLE,
        START,
        MODE,
        DEAD,
        SHOOTER_POWER,
        COOLING_HEAT1,
        COOLING_HEAT2,
        COOLING_LIMIT1,
        COOLING_LIMIT2,
        SPEED_LIMIT1,
        SPEED_LIMIT2,
    } can_bridge_cmd;

    typedef struct {
        uint8_t id;
        union {
            float data_float;
            int data_int;
            bool data_bool;
        };
    } bridge_data_t;

    class CanBridge {
      public:
        CanBridge(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
        void UpdateData(const uint8_t data[]);
        void TransmitOutput();

        bridge_data_t cmd;
        float vx = 0;
        float vy = 0;
        float relative_angle = 0;
        bool start = false;
        int mode = 0;
        bool dead = false;
        bool shooter_power = false;
        float cooling_heat1 = 0;
        float cooling_heat2 = 0;
        float cooling_limit1 = 0;
        float cooling_limit2 = 0;
        float speed_limit1 = 0;
        float speed_limit2 = 0;

      private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
    };

}  // namespace bsp