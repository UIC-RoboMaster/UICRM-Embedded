#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
#define LEFT_MOTOR_PWM_CHANNEL 1
#define RIGHT_MOTOR_PWM_CHANNEL 2
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 500
#define SNAIL_IDLE_THROTTLE 1080

control::MotorPWMBase* motor1;
control::MotorPWMBase* motor2;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
    dbus = new remote::DBUS(&huart3);
    print_use_uart(&huart4);
    motor1 = new control::MotorPWMBase(&htim8, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                       MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
    motor2 = new control::MotorPWMBase(&htim8, RIGHT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                       MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
    motor1->SetOutput(0);
    motor2->SetOutput(0);
    // Snail need to be run at idle throttle for some
    osDelay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    RampSource ramp_1 = RampSource(0, 0, 450, 0.001);
    RampSource ramp_2 = RampSource(0, 0, 450, 0.001);
    int current = 0;
    int state = 0;
    int8_t last_state = remote::MID;
    while (true) {
        if (dbus->swl == remote::UP) {
            if (last_state == remote::MID)
                last_state = remote::UP;
        } else if (dbus->swl == remote::MID) {
            if (last_state == remote::UP) {
                last_state = remote::MID;

                if (state == 0) {
                    state = 1;
                    current = 100;
                } else {
                    state = 0;
                    current = -100;
                }
            }
        }
        motor1->SetOutput((int16_t)ramp_1.Calc((float)current));
        motor2->SetOutput((int16_t)ramp_2.Calc((float)current));
        osDelay(1);
    }
}
