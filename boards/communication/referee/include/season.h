/*###########################################################
# Copyright (c) 2026-2027. BNBU/UIC RoboMaster             #
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
#pragma once

/*
 * 赛季选择入口。
 *
 * 由各 program 在自己的 CMakeLists.txt 中指定，未指定时默认 2027：
 *
 *   uicrm_add_arm_executable(${PROJECT_NAME}
 *           TARGET DJI_Board_TypeA_general
 *           ...
 *           DEFINES HAS_REFEREE REFEREE_SEASON=2027)
 *
 * 业务代码统一通过 referee_comm::season 引用具体实现，不直接写 v2026 / v2027，
 * 切换赛季时只改 program 的 CMakeLists，不动任何业务源文件：
 *
 *   #include "season.h"
 *   referee_comm::season::Protocol protocol;
 *   referee_comm::Referee referee(&huart3, protocol);
 */

#ifndef REFEREE_SEASON
#define REFEREE_SEASON 2027
#endif

#if REFEREE_SEASON == 2026
#include "seasons/2026/protocol.h"
namespace referee_comm {
    namespace season = v2026;
}  // namespace referee_comm
#elif REFEREE_SEASON == 2027
#include "seasons/2027/protocol.h"
namespace referee_comm {
    namespace season = v2027;
}  // namespace referee_comm
#else
#error "Unsupported REFEREE_SEASON, expected 2026 or 2027"
#endif
