//
// Created by Administrator on 2024/11/22.
//
#include "RM_Debug_control_model.h"

#define Debug_true 1
#define Debug_false 0

#define Control_true 1
#define Control_false 0

communication::DebugModel* CAR_DEBUG = nullptr;

char s[50];

void init_rm_debug() {
    print("RM_INIT>>\n");
    CAR_DEBUG = new communication::DebugModel();
    CAR_DEBUG->RM_Debug_mode.debug_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.gimbal_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.chassis_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.shooter_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.supercapacitor_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.vision_debug_mode = Debug_false;

    CAR_DEBUG->RM_Control_model.chassis_control_model = Control_false;
    CAR_DEBUG->RM_Control_model.gimbal_control_model = Control_false;
    CAR_DEBUG->RM_Control_model.shooter_control_model = Control_false;
}