/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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
#include "main.h"

namespace driver {
    /**
     * @brief DJI通用电机的标准接口
     */
    /**
     * @brief Basic Interface for DJI Motor
     */
    class MotorBase {
      public:
        MotorBase() : output_(0) {
        }
        virtual ~MotorBase() {
        }

        virtual void SetOutput(int16_t val) {
            output_ = val;
        }

        virtual int16_t GetOutput() {
            return output_;
        }

      protected:
        int16_t output_;
    };
};  // namespace driver
