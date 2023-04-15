#include "ui_task.h"

osThreadId_t uiTaskHandle;
communication::UserInterface* UI = nullptr;
communication::ChassisGUI* chassisGUI = nullptr;
communication::CrossairGUI* crossairGui = nullptr;
communication::GimbalGUI* gimbalGUI = nullptr;
void uiTask(void* arg) {
    UNUSED(arg);

    osDelay(3000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(UI_OS_DELAY);
    }
    UI->SetID(referee->game_robot_status.robot_id);
    osDelay(UI_OS_DELAY);



    communication::graphic_data_t graphBarFrame;
    communication::graphic_data_t graphBar;
    communication::graphic_data_t graphPercent;
    communication::graphic_data_t graphDiag;
    communication::graphic_data_t graphMode;
    communication::graphic_data_t graphWheel;
    // Initialize chassis GUI
    chassisGUI = new communication::ChassisGUI(UI);
    osDelay(100);
    chassisGUI->Init2();
    osDelay(100);
    // Initialize crosshair GUI
    crossairGui = new communication::CrossairGUI(UI);
    osDelay(100);

    // Initialize supercapacitor GUI
    UI->CapGUIInit(&graphBarFrame, &graphBar);
    UI->GraphRefresh(2, graphBarFrame, graphBar);
    osDelay(100);

    // Initialize Supercapacitor string GUI
    UI->CapGUICharInit(&graphPercent);
    UI->CharRefresh(graphPercent, UI->getPercentStr(), UI->getPercentLen());
    osDelay(100);

    // Initialize Gimbal GUI
    gimbalGUI = new communication::GimbalGUI(UI);
    osDelay(100);
    gimbalGUI->Init2();

    // Initialize self-diagnosis GUI
    char diagStr[29] = "";
    UI->DiagGUIInit(&graphDiag, 29);
    UI->CharRefresh(graphDiag, diagStr, 2);
    osDelay(100);

    // Initialize current mode GUI
    char followModeStr[15] = "FOLLOW MODE";
    // char spinModeStr[15] = "SPIN  MODE";
    uint32_t modeColor = UI_Color_Orange;
    UI->ModeGUIInit(&graphMode);
    UI->CharRefresh(graphMode, followModeStr, sizeof followModeStr);
    osDelay(100);



    // Initialize flywheel status GUI
    char wheelOnStr[15] = "FLYWHEEL ON";
    char wheelOffStr[15] = "FLYWHEEL OFF";
    UI->WheelGUIInit(&graphWheel);
    UI->CharRefresh(graphWheel, wheelOffStr, sizeof wheelOffStr);
    osDelay(100);

    float relative_angle = 0;
    float pitch_angle = 0;
    float power_percent = 1;
    while (true) {

        // Update chassis GUI
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
        pitch_angle = pitch_motor->GetThetaDelta(gimbal_param->pitch_offset_);
        chassisGUI->Update(chassis_vx/chassis_vx_max,chassis_vy/chassis_vy_max,relative_angle);
        osDelay(UI_OS_DELAY);

        power_percent = battery_vol->GetBatteryPercentage();
        // Update Power GUI
        UI->CapGUIUpdate(power_percent);
        UI->GraphRefresh(1, graphBar);
        osDelay(UI_OS_DELAY);

        // Update supercapacitor string GUI
        UI->CapGUICharUpdate();
        UI->CharRefresh(graphPercent, UI->getPercentStr(), UI->getPercentLen());
        osDelay(UI_OS_DELAY);

        // Update Gimbal GUI
        gimbalGUI->Update(pitch_diff/PI, yaw_diff/PI, pitch_angle, relative_angle,imu->CaliDone());
        osDelay(UI_OS_DELAY);

        // Update current mode GUI
        char modeStr[30];
        switch (remote_mode) {
            case REMOTE_MODE_STOP:
                strcpy(modeStr, "STOP MODE     ");
                modeColor = UI_Color_Purplish_red;
                break;
            case REMOTE_MODE_MANUAL:
                strcpy(modeStr, "FOLLOW MODE   ");
                modeColor = UI_Color_Green;
                break;
            case REMOTE_MODE_SPIN:
                strcpy(modeStr, "SPIN MODE     ");
                modeColor = UI_Color_Orange;
                break;
            case REMOTE_MODE_ADVANCED:
                strcpy(modeStr, "ADVANCED MODE ");
                modeColor = UI_Color_Green;
                break;
            default:
                strcpy(modeStr, "UNKNOWN MODE   ");
                modeColor = UI_Color_Purplish_red;
                break;
        }

        UI->ModeGuiUpdate(&graphMode, modeColor);
        UI->CharRefresh(graphMode, modeStr, 15);
        osDelay(UI_OS_DELAY);

        // Update wheel status GUI
        char* wheelStr = shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? wheelOnStr : wheelOffStr;
        uint32_t wheelColor =
            shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? UI_Color_Pink : UI_Color_Green;
        UI->WheelGUIUpdate(&graphWheel, wheelColor);
        UI->CharRefresh(graphWheel, wheelStr, 15);
        osDelay(UI_OS_DELAY);

        // Update self-diagnosis messages

        // clear self-diagnosis messages
        if (dbus->keyboard.bit.C) {
            for (int i = 1; i <= UI->getMessageCount(); ++i) {
                UI->DiagGUIClear(UI, referee, &graphDiag, i);
                osDelay(UI_OS_DELAY);
            }
        }
    }
}

void init_ui() {
    UI = new communication::UserInterface(referee_uart, referee);
}
