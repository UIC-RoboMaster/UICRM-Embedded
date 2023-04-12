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

    communication::package_t frame;
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
//    communication::graphic_data_t graphDist;
    communication::graphic_data_t graphWheel;
    // Initialize chassis GUI
    UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphCali, &graphEmpty2);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                     graphCali, graphEmpty2);
    referee->PrepareUIContent(communication::FIVE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(100);

    // Initialize crosshair GUI
    UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                     &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                     graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                     graphCrosshair7);
    referee->PrepareUIContent(communication::SEVEN_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(100);

    // Initialize supercapacitor GUI
    UI->CapGUIInit(&graphBarFrame, &graphBar);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
    referee->PrepareUIContent(communication::DOUBLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(100);

    // Initialize Supercapacitor string GUI
    UI->CapGUICharInit(&graphPercent);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                    UI->getPercentLen());
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(100);

    // Initialize self-diagnosis GUI
    char diagStr[30] = "";
    UI->DiagGUIInit(&graphDiag, 30);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDiag, diagStr, 2);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(100);

    // Initialize current mode GUI
    char followModeStr[15] = "FOLLOW MODE";
    // char spinModeStr[15] = "SPIN  MODE";
    uint32_t modeColor = UI_Color_Orange;
    UI->ModeGUIInit(&graphMode);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, followModeStr,
                    sizeof followModeStr);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
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
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelOffStr,
                    sizeof wheelOffStr);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
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
        UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                         graphCali, graphEmpty2);
        referee->PrepareUIContent(communication::FIVE_GRAPH);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
        osDelay(UI_OS_DELAY);

        // Update supercapacitor GUI
        UI->CapGUIUpdate(std::abs(sin(j)));
        UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
        referee->PrepareUIContent(communication::SINGLE_GRAPH);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
        j += 0.1;
        osDelay(UI_OS_DELAY);

        // Update supercapacitor string GUI
        UI->CapGUICharUpdate();
        UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                        UI->getPercentLen());
        referee->PrepareUIContent(communication::CHAR_GRAPH);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
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
        UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, modeStr, 15);
        referee->PrepareUIContent(communication::CHAR_GRAPH);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
        osDelay(UI_OS_DELAY);

        // Update distance GUI
        //     uint32_t distColor = UI_Color_Cyan;
        //     float currDist = LIDAR->distance / 1000.0;
        //     if (currDist < 60) {
        //       snprintf(distanceStr, 15, "%.2f m", currDist);
        //       distColor = UI_Color_Cyan;
        //     } else {
        //       snprintf(distanceStr, 15, "ERROR");
        //       distColor = UI_Color_Pink;
        //     }
        //     UI->DistanceGUIUpdate(&graphDist, distColor);
        //     UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr, 15);
        //     referee->PrepareUIContent(communication::CHAR_GRAPH);
        //     frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        //     referee_uart->Write(frame.data, frame.length);
        //     osDelay(UI_TASK_DELAY);

        //    // Update lid status GUI
        //    char lidStr[15] = lidFlag ? lidOpenStr : lidCloseStr;
        //    uint32_t lidColor = lidFlag ? UI_Color_Pink : UI_Color_Green;
        //    UI->LidGuiUpdate(&graphLid, lidColor);
        //    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidStr, 15);
        //    referee->PrepareUIContent(communication::CHAR_GRAPH);
        //    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        //    referee_uart->Write(frame.data, frame.length);
        //    osDelay(UI_TASK_DELAY);

        // Update wheel status GUI
        char* wheelStr = shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? wheelOnStr : wheelOffStr;
        uint32_t wheelColor = shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? UI_Color_Pink : UI_Color_Green;
        UI->WheelGUIUpdate(&graphWheel, wheelColor);
        UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelStr, 15);
        referee->PrepareUIContent(communication::CHAR_GRAPH);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
        osDelay(UI_OS_DELAY);

        // Update self-diagnosis messages



        // clear self-diagnosis messages
        if (dbus->keyboard.bit.C) {
            for (int i = 1; i <= UI->getMessageCount(); ++i) {
                UI->DiagGUIClear(UI, referee, &graphDiag, i);
                frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
                referee_uart->Write(frame.data, frame.length);
                osDelay(UI_OS_DELAY);
            }
        }
    }

}

void init_ui() {
    UI = new communication::UserInterface();
}
