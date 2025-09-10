//
// Created by Administrator on 2024/11/22.
//
#pragma once

#include "bsp_print.h"

namespace communication {
    /* ===== Debug_mode 0x0601 ===== */
    typedef struct {
        bool debug_debug_mode;
        bool gimbal_debug_mode;
        bool chassis_debug_mode;
        bool shooter_debug_mode;
        bool supercapacitor_debug_mode;
        bool vision_debug_mode;

    } RM_Debug_mode_t;

    /* ===== control_model  0x0602 ===== */
    typedef struct {
        bool gimbal_control_model;
        bool chassis_control_model;
        bool shooter_control_model;

    } RM_Control_model_t;

    class DebugModel {
      public:
        RM_Debug_mode_t RM_Debug_mode{};
        RM_Control_model_t RM_Control_model{};
    };

}  // namespace communication

void init_rm_debug();

/* namespace communication */