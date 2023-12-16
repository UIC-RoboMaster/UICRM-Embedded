/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "main.h"
#include "bsp_can.h"

namespace driver {
    typedef enum{
        CYBERGEAR_MODE_CURRENT = 3,
        CYBERGEAR_MODE_SPEED = 2,
        CYBERGEAR_MODE_POSITION = 1,
        CYBERGEAR_MODE_MIT = 0,
    }cybergear_mode_e;
    typedef struct {
        uint32_t id:8;
        uint32_t data:16;
        uint32_t mode:5;
        uint32_t res:3;
    }cybergear_extid_t;

    typedef enum{
        CYBERGEAR_ERROR_NONE = 0,
        CYBERGEAR_ERROR_BATTERY_LOW = 1,
        CYBERGEAR_ERROR_OVER_CURRENT = 2,
        CYBERGEAR_ERROR_OVER_TEMPERATURE = 3,
        CYBERGEAR_ERROR_MAGNETIC = 4,
        CYBERGEAR_ERROR_ENCODER = 5,
        CYBERGEAR_ERROR_NO_CAILBRATION = 6,
    }cybergear_error_e;
    typedef enum{
        CYBERGEAR_STATUS_RESET = 0,
        CYBERGEAR_STATUS_CALI = 1,
        CYBERGEAR_STATUS_NORMAL = 2,
    }cybergear_status_e;
    class CyberGear {
      public:
        CyberGear(bsp::CAN* can, uint8_t tx_id, uint8_t rx_id);
        ~CyberGear();

        void UpdateData(const uint8_t data[],const uint32_t ext_id);


        void MotorEnable();
        void MotorDisable();

        void Reset();

        // void ReadIndex(uint16_t index);

        void WriteIndex(uint16_t index, uint8_t data);

        void WriteIndex(uint16_t index, uint16_t data);

        void WriteIndex(uint16_t index, float data);

        void SetMode(cybergear_mode_e mode);

        void SetOutput(float output);

        void SetOutout(float position, float velocity);

        void SetOutput(float position,float velocity, float torque, float kp, float kd);

        float GetTheta() const;

        float GetThetaDelta(const float target)const;

        float GetMITTheta() const;

        float GetMITThetaDelta(const float target)const;

        float GetOmega() const;

        float GetOmegaDelta(const float target)const;

        float GetTorque() const;

        float GetTemp() const;

        void PrintData();

      private:
        bsp::CAN* can_;
        uint8_t tx_id_;
        uint8_t rx_id_;
        cybergear_extid_t tx_ext_id_= {
            .id = 0,
            .data = 0,
            .mode = 0,
            .res = 0,
        };
        cybergear_extid_t rx_ext_id_= {
            .id = 0,
            .data = 0,
            .mode = 0,
            .res = 0,
        };
        uint8_t tx_data_[8]={0};


        cybergear_mode_e mode_=CYBERGEAR_MODE_CURRENT;

        float theta_=0;
        float mit_theta_=0;
        float omega_=0;
        float torque_=0;
        float temperature_=0;
        float current_=0;

        uint8_t error_code_=0;
        uint8_t status_=0;

        void TransmitData();

        static int16_t float_to_uint(float x, float x_min, float x_max, int bits);

        static float uint_to_float(int x, float x_min, float x_max, int bits);

    };

};