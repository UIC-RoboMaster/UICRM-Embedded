#pragma once
#include "bsp_can.h"

namespace control {

    typedef struct {
        float input_voltage;
        float supercap_voltage;
        float input_current;
        float target_power;
    } __packed cap_message_t;

    class SuperCap {
      public:
        SuperCap(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
        void UpdateData(const uint8_t data[]);
        void SetTargetPower(float power);
        volatile bool connection_flag_ = false;
        float GetPercent();

        cap_message_t info;
      private:
        bsp::CAN* can_;
        uint16_t supercap_rx_id_;
        uint16_t supercap_tx_id_;
        float percent_;
    };

}