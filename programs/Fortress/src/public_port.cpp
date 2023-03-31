#include "public_port.h"
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
void init_can() {
    can1 = new bsp::CAN(&hcan1, 0x201, true);
    can2 = new bsp::CAN(&hcan2, 0x205, false);
}