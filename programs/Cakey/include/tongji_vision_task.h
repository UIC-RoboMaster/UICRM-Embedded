// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Sarzn on 2026/6/10.
//

#ifndef UICRM_TONGJI_VISION_TASK_H
#define UICRM_TONGJI_VISION_TASK_H

#pragma once
#include "cmsis_os.h"
#include "tongji_vision.h"

extern driver::TongjiVision* tongji_vision;
extern osThreadId_t tongjiVisionTaskHandle;
extern const osThreadAttr_t tongjiVisionTaskAttribute;

void init_tongji_vision();
void tongjiVisionTask(void* arg);

#endif