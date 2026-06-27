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
// Created by Sarzn on 2026/4/14.
//

#ifndef UICRM_CONFIG_H
#define UICRM_CONFIG_H

#include <sys/types.h>
// 这是屏幕坐标，请使用 pixpin 校准后填入
static constexpr uint16_t POS_50_X = 1160;  // 50 发
static constexpr uint16_t POS_50_Y = 560;
static constexpr uint16_t POS_100_X = 1210;  // 100 发
static constexpr uint16_t POS_100_Y = 560;
static constexpr uint16_t POS_200_X = 1210;  // 200 发 （点两次）
static constexpr uint16_t POS_200_Y = 560;
static constexpr uint16_t BUY_X = 960;  // 购弹按钮
static constexpr uint16_t BUY_Y = 670;
static constexpr uint16_t CONFIRM_BUY_X = 860;  // 确认按钮
static constexpr uint16_t CONFIRM_BUY_Y = 560;

static constexpr uint32_t CLICK_DELAY_MS = 25;  // 每次按下要等待的时间
static constexpr uint32_t LOOP_DELAY_MS = 10;
#endif  // UICRM_CONFIG_H