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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
#define LEFT_MOTOR_PWM_CHANNEL 1
#define RIGHT_MOTOR_PWM_CHANNEL 4
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 500
#define SNAIL_IDLE_THROTTLE 1080

//driver::MotorPWMBase* motor1;
//driver::MotorPWMBase* motor2;

static bsp::PWM* pwm1 = nullptr;
static bsp::PWM* pwm2 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart8);
//    motor1 = new driver::MotorPWMBase(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
//                                      MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
//    motor2 = new driver::MotorPWMBase(&htim1, RIGHT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
//                                      MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
//    motor1->SetOutput(0);
//    motor2->SetOutput(0);
    // Snail need to be run at idle throttle for some
    pwm1 = new bsp::PWM(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 1500);
    pwm2 = new bsp::PWM(&htim1, RIGHT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 1500);
    pwm1->Start();
    pwm2->Start();
    HAL_Delay(5000);
    pwm1->SetPulseWidth(500);
    pwm2->SetPulseWidth(500

                        );
    HAL_Delay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
//    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
//
//    int current = 0;
    while (true) {
//        if (key.Read() == 0) {
//            osDelay(30);
//            if (key.Read() == 1)
//                continue;
//            while (key.Read() == 0) {
//                osDelay(30);
//            }
//            if (current == 0) {
//                current = 500;
//                motor1->SetOutput(current);
//                motor2->SetOutput(current);
//            } else {
//                current = 0;
//                motor1->SetOutput(current);
//                motor2->SetOutput(current);
//            }
//            osDelay(200);
//        }
    }
}
