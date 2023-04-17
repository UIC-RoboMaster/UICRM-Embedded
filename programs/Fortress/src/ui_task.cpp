#include "ui_task.h"

osThreadId_t uiTaskHandle;
communication::UserInterface* UI = nullptr;
communication::ChassisGUI* chassisGUI = nullptr;
communication::CrossairGUI* crossairGui = nullptr;
communication::GimbalGUI* gimbalGUI = nullptr;
communication::CapGUI* batteryGUI = nullptr;
communication::StringGUI* modeGUI = nullptr;
communication::StringGUI* wheelGUI = nullptr;
communication::StringGUI* boostGUI = nullptr;
void uiTask(void* arg) {
    UNUSED(arg);

    osDelay(3000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(UI_OS_DELAY);
    }
    UI->SetID(referee->game_robot_status.robot_id);
    osDelay(UI_OS_DELAY);

    communication::graphic_data_t graphEmpty1;
    UI->CircleDraw(&graphEmpty1, "E1", UI_Graph_Del, 0, UI_Color_Green, 0, 0, 0, 0);
    communication::graphic_data_t graphEmpty2;
    UI->CircleDraw(&graphEmpty2, "E2", UI_Graph_Del, 0, UI_Color_Green, 0, 0, 0, 0);

    communication::graphic_data_t graphDiag;

    // Initialize chassis GUI
    chassisGUI = new communication::ChassisGUI(UI);
    osDelay(110);
    chassisGUI->Init2();
    osDelay(110);
    // Initialize crosshair GUI
    crossairGui = new communication::CrossairGUI(UI);
    osDelay(110);

    // Initialize supercapacitor GUI
    char batteryStr[15] = "BATTERY";
    batteryGUI = new communication::CapGUI(UI, batteryStr);
    osDelay(110);
    batteryGUI->InitName();
    osDelay(110);

    // Initialize Gimbal GUI
    gimbalGUI = new communication::GimbalGUI(UI);
    osDelay(110);
    gimbalGUI->Init2();

    // Initialize self-diagnosis GUI
    char diagStr[29] = "";
    UI->DiagGUIInit(&graphDiag, 29);
    UI->CharRefresh(graphDiag, diagStr, 2);
    osDelay(110);

    // Initialize current mode GUI
    char followModeStr[15] = "FOLLOW MODE";
    int8_t modeColor = UI_Color_Orange;
    modeGUI = new communication::StringGUI(UI, followModeStr, 1230, 45, modeColor);
    // Initialize flywheel status GUI
    char wheelOnStr[15] = "FLYWHEEL ON";
    char wheelOffStr[15] = "FLYWHEEL OFF";
    wheelGUI = new communication::StringGUI(UI, wheelOffStr, 1500, 430, UI_Color_Pink);
    char boostModeStr[15] = "BOOST!";
    char boostOffStr[15] = " ";
    boostGUI = new communication::StringGUI(UI, boostOffStr, 870, 630, UI_Color_Pink, 30);
    communication::graphic_data_t graphMode;
    communication::graphic_data_t graphWheel;
    communication::graphic_data_t graphBoost;

    graphMode = modeGUI->InitBulk();
    graphWheel = wheelGUI->InitBulk();
    graphBoost = boostGUI->InitBulk();
    UI->GraphRefresh(5, graphMode, graphWheel, graphBoost, graphEmpty1, graphEmpty2);
    osDelay(110);
    modeGUI->InitString();
    osDelay(110);
    wheelGUI->InitString();
    osDelay(110);
    boostGUI->InitString();
    float relative_angle = 0;
    float pitch_angle = 0;
    float power_percent = 1;
    RemoteMode last_mode = REMOTE_MODE_KILL;
    ShootFricMode last_fric_mode = SHOOT_FRIC_MODE_STOP;
    BoolEdgeDetector boostEdgeDetector(false);
    while (true) {
        // Update chassis GUI
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
        pitch_angle = pitch_motor->GetThetaDelta(gimbal_param->pitch_offset_);
        chassisGUI->Update(chassis_vx / chassis_vx_max, chassis_vy / chassis_vy_max,
                           relative_angle);
        osDelay(UI_OS_DELAY);

        power_percent = battery_vol->GetBatteryPercentage();

        batteryGUI->Update(power_percent);
        osDelay(UI_OS_DELAY);

        // Update Gimbal GUI
        gimbalGUI->Update(pitch_diff * 200, -yaw_diff * 200, pitch_angle, relative_angle,
                          imu->CaliDone());
        osDelay(UI_OS_DELAY);

        // Update current mode GUI
        if (remote_mode != last_mode) {
            char modeStr[30];
            switch (remote_mode) {
                case REMOTE_MODE_STOP:
                    strcpy(modeStr, "STOP MODE     ");
                    modeColor = UI_Color_Purplish_red;
                    break;
                case REMOTE_MODE_FOLLOW:
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

            modeGUI->Update(modeStr, modeColor);
            osDelay(UI_OS_DELAY);
        }

        // Update wheel status GUI
        if (last_fric_mode != shoot_fric_mode) {
            char* wheelStr = shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? wheelOnStr : wheelOffStr;
            uint32_t wheelColor =
                shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED ? UI_Color_Pink : UI_Color_Green;
            wheelGUI->Update(wheelStr, wheelColor);
            osDelay(UI_OS_DELAY);
        }
        boostEdgeDetector.input(chassis_boost_flag);
        if (boostEdgeDetector.edge()) {
            char* boostStr = chassis_boost_flag ? boostModeStr : boostOffStr;
            boostGUI->Update(boostStr, UI_Color_Pink);
            osDelay(UI_OS_DELAY);
        }
        last_mode = remote_mode;
        last_fric_mode = shoot_fric_mode;

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
