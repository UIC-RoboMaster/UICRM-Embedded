#include "ui_task.h"



osThreadId_t uiTaskHandle;
communication::UserInterface* UI = nullptr;
void uiTask(void* arg) {
    UNUSED(arg);



    osDelay(3000);
    while(remote_mode == REMOTE_MODE_KILL) {
        osDelay(UI_OS_DELAY);
    }
    UI->SetID(referee->game_robot_status.robot_id);
    osDelay(UI_OS_DELAY);

    communication::graphic_data_t graphGimbal;
    communication::graphic_data_t graphChassis;
    communication::graphic_data_t graphArrow;
    communication::graphic_data_t graphCali;
    communication::graphic_data_t graphEmpty2;
    communication::graphic_data_t graphCrosshair1;
    communication::graphic_data_t graphCrosshair2;
    communication::graphic_data_t graphCrosshair3;
    communication::graphic_data_t graphCrosshair4;
    communication::graphic_data_t graphCrosshair5;
    communication::graphic_data_t graphCrosshair6;
    communication::graphic_data_t graphCrosshair7;
    communication::graphic_data_t graphBarFrame;
    communication::graphic_data_t graphBar;
    communication::graphic_data_t graphPercent;
    communication::graphic_data_t graphDiag;
    communication::graphic_data_t graphMode;
    communication::graphic_data_t graphWheel;
    // Initialize chassis GUI
    UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphCali, &graphEmpty2);
    UI->GraphRefresh(5, graphChassis, graphArrow, graphGimbal,
                     graphCali, graphEmpty2);
    osDelay(100);

    // Initialize crosshair GUI
    UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                     &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
    UI->GraphRefresh(7, graphCrosshair1, graphCrosshair2,
                     graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                     graphCrosshair7);
    osDelay(100);

    // Initialize supercapacitor GUI
    UI->CapGUIInit(&graphBarFrame, &graphBar);
    UI->GraphRefresh(2, graphBarFrame, graphBar);
    osDelay(100);

    // Initialize Supercapacitor string GUI
    UI->CapGUICharInit(&graphPercent);
    UI->CharRefresh(graphPercent, UI->getPercentStr(),
                    UI->getPercentLen());
    osDelay(100);

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
    UI->CharRefresh(graphMode, followModeStr,
                    sizeof followModeStr);
    osDelay(100);



    // Initialize distance GUI
//    char distanceStr[15] = "0.0";
//    UI->DistanceGUIInit(&graphDist);
//    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr,
//                    sizeof distanceStr);
//    referee->PrepareUIContent(communication::CHAR_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(UI_OS_DELAY);

    // TODO: add lid UI in the future

    //  // Initialize lid status GUI
    //  char lidOpenStr[15] = "LID OPENED";
    //  char lidCloseStr[15] = "LID CLOSED";
    //  UI->LidGUIInit(&graphLid);
    //  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidOpenStr, sizeof
    //  lidOpenStr); referee->PrepareUIContent(communication::CHAR_GRAPH); frame =
    //  referee->Transmit(communication::STUDENT_INTERACTIVE); referee_uart->Write(frame.data,
    //  frame.length); osDelay(UI_TASK_DELAY);

    // Initialize flywheel status GUI
    char wheelOnStr[15] = "FLYWHEEL ON";
    char wheelOffStr[15] = "FLYWHEEL OFF";
    UI->WheelGUIInit(&graphWheel);
    UI->CharRefresh(graphWheel, wheelOffStr,sizeof wheelOffStr);
    osDelay(100);

    float j = 1;
    float relative_angle = 0;
    bool calibration_flag = false;
    while (true) {
        //     lidar_flag = LIDAR->startMeasure();

        // Update chassis GUI
        calibration_flag = imu->CaliDone();
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
        UI->ChassisGUIUpdate(relative_angle, calibration_flag);
        UI->GraphRefresh(5, graphChassis, graphArrow, graphGimbal,
                         graphCali, graphEmpty2);
        osDelay(UI_OS_DELAY);

        // Update supercapacitor GUI
        UI->CapGUIUpdate(std::abs(arm_sin_f32(j)));
        UI->GraphRefresh(1, graphBar);
        j += 0.1;
        osDelay(UI_OS_DELAY);

        // Update supercapacitor string GUI
        UI->CapGUICharUpdate();
        UI->CharRefresh(graphPercent, UI->getPercentStr(),
                        UI->getPercentLen());
        osDelay(UI_OS_DELAY);

        // Update current mode GUI
        char modeStr[30];
        switch(remote_mode){
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
        uint32_t wheelColor = shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? UI_Color_Pink : UI_Color_Green;
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
    UI = new communication::UserInterface(referee_uart,referee);
}
