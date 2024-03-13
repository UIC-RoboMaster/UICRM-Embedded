/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

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
communication::DiagGUI* diagGUI = nullptr;

void UI_Delay(uint32_t delay) {
    osDelay(delay);
}

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
    diagGUI = new communication::DiagGUI(UI);

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
    BoolEdgeDetector* boostEdgeDetector = new BoolEdgeDetector(false);
    BoolEdgeDetector* c_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* v_edge = new BoolEdgeDetector(false);

    BoolEdgeDetector* fl_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* fr_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* bl_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* br_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* yaw_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* pitch_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* steer_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* dbus_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* imu_cali_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* imu_temp_edge = new BoolEdgeDetector(false);
    while (true) {
        // Update chassis GUI
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
        pitch_angle = pitch_motor->GetThetaDelta(gimbal_param->pitch_offset_);
        chassisGUI->Update(chassis_vx / chassis_vx_max, chassis_vy / chassis_vy_max,
                           relative_angle);
        osDelay(UI_OS_DELAY);

        power_percent = 1.0f;

        batteryGUI->Update(power_percent);
        osDelay(UI_OS_DELAY);

        // Update Gimbal GUI
        gimbalGUI->Update(pitch_diff * 200, -yaw_diff * 200, pitch_angle, relative_angle, true);
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
                case REMOTE_MODE_MANUAL:
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
        if (last_fric_mode != shoot_flywheel_mode) {
            char* wheelStr =
                shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED ? wheelOnStr : wheelOffStr;
            uint32_t wheelColor =
                shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED ? UI_Color_Pink : UI_Color_Green;
            wheelGUI->Update(wheelStr, wheelColor);
            osDelay(UI_OS_DELAY);
        }
        boostEdgeDetector->input(chassis_boost_flag);
        if (boostEdgeDetector->edge()) {
            char* boostStr = chassis_boost_flag ? boostModeStr : boostOffStr;
            boostGUI->Update(boostStr, UI_Color_Pink);
            osDelay(UI_OS_DELAY);
        }
        last_mode = remote_mode;
        last_fric_mode = shoot_flywheel_mode;

        // Update self-diagnosis messages
        {
            fl_motor_check_edge->input(selftest.fl_motor);
            fr_motor_check_edge->input(selftest.fr_motor);
            bl_motor_check_edge->input(selftest.bl_motor);
            br_motor_check_edge->input(selftest.br_motor);
            yaw_motor_check_edge->input(selftest.yaw_motor);
            pitch_motor_check_edge->input(selftest.pitch_motor);
            steer_motor_check_edge->input(selftest.steering_motor);
            dbus_edge->input(selftest.dbus);
            imu_cali_edge->input(selftest.imu_cali);
            imu_temp_edge->input(selftest.imu_temp);
            if (fl_motor_check_edge->negEdge()) {
                strcpy(diagStr, "FL MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (fr_motor_check_edge->negEdge()) {
                strcpy(diagStr, "FR MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (bl_motor_check_edge->negEdge()) {
                strcpy(diagStr, "BL MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (br_motor_check_edge->negEdge()) {
                strcpy(diagStr, "BR MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (yaw_motor_check_edge->negEdge()) {
                strcpy(diagStr, "YAW MOTOR OFFLINE    ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (pitch_motor_check_edge->negEdge()) {
                strcpy(diagStr, "PITCH MOTOR OFFLINE  ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (steer_motor_check_edge->negEdge()) {
                strcpy(diagStr, "STEER MOTOR OFFLINE  ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (dbus_edge->negEdge()) {
                strcpy(diagStr, "DBUS OFFLINE         ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (imu_cali_edge->posEdge()) {
                strcpy(diagStr, "IMU CALIBRATION DONE");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Green);
            }
            if (imu_temp_edge->posEdge()) {
                strcpy(diagStr, "IMU TEMP NOT SAFE   ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
        }

        // clear self-diagnosis messages
        if (selftest.dbus) {
            v_edge->input(dbus->keyboard.bit.V);
        } else {
            v_edge->input(refereerc->remote_control.keyboard.bit.V);
        }

        if (v_edge->posEdge()) {
            osDelay(110);
            chassisGUI->Delete2();
            osDelay(110);
            chassisGUI->Delete();
            osDelay(110);
            gimbalGUI->Delete2();
            osDelay(110);
            gimbalGUI->Delete();
            osDelay(110);
            crossairGui->Delete();
            osDelay(110);
            batteryGUI->DeleteName();
            osDelay(110);
            batteryGUI->Delete();
            osDelay(110);
            graphMode = modeGUI->DeleteBulk();
            graphBoost = boostGUI->DeleteBulk();
            graphWheel = wheelGUI->DeleteBulk();
            UI->GraphRefresh(5, graphMode, graphWheel, graphBoost, graphEmpty1, graphEmpty2);
            osDelay(110);
            diagGUI->Clear(UI_Delay);
            osDelay(110);
            chassisGUI->Init();
            osDelay(110);
            chassisGUI->Init2();
            osDelay(110);
            gimbalGUI->Init();
            osDelay(110);
            gimbalGUI->Init2();
            osDelay(110);
            crossairGui->Init();
            osDelay(110);
            batteryGUI->Init();
            osDelay(110);
            batteryGUI->InitName();
            osDelay(110);
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
            osDelay(110);
            continue;
        }
        if (selftest.dbus) {
            c_edge->input(dbus->keyboard.bit.C);
        } else {
            c_edge->input(refereerc->remote_control.keyboard.bit.C);
        }
        if (c_edge->posEdge()) {
            diagGUI->Clear(UI_Delay);
        }
    }
}

void init_ui() {
    UI = new communication::UserInterface(referee_uart, referee);
}
