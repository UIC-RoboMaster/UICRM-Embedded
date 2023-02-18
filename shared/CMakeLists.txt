# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2022                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

# Eigen interface library
add_library(eigen3 INTERFACE)
target_include_directories(eigen3 SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.3.9)

file(GLOB_RECURSE BSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cpp)
file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/*.cpp)
file(GLOB_RECURSE THIRD_PARTY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/*.c)

set(UICRM_SOURCES ${BSP_SOURCES} ${LIB_SOURCES} ${THIRD_PARTY_SOURCES})
set(UICRM_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_uicrm
    TARGET DJI_Board_TypeC_general
    SOURCES ${UICRM_SOURCES}
    INCLUDES ${UICRM_INCLUDES}
    DEPENDS eigen3)
