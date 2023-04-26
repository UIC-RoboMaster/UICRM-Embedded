#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeC datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static control::Motor3508* motor1 = nullptr;
static control::Motor3508* motor2 = nullptr;
static float* pid1_param = nullptr;
static float* pid2_param = nullptr;
static control::FlyWheelMotor* flywheel1 = nullptr;
static control::FlyWheelMotor* flywheel2 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart6);
    can1 = new bsp::CAN(&hcan1, 0x201, true);
    motor1 = new control::Motor3508(can1, 0x201);
    motor2 = new control::Motor3508(can1, 0x202);

    pid1_param = new float[3]{150, 1, 0.15};
    pid2_param = new float[3]{150, 1, 0.15};
    control::flywheel_t flywheel1_data = {
        .motor = motor1,
        .max_speed = 400 * PI,
        .omega_pid_param = pid1_param,
        .is_inverted = false,
    };control::flywheel_t flywheel2_data = {
        .motor = motor2,
        .max_speed = 400 * PI,
        .omega_pid_param = pid2_param,
        .is_inverted = true,
    };
    flywheel1 = new control::FlyWheelMotor(flywheel1_data);
    flywheel2 = new control::FlyWheelMotor(flywheel2_data);
    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    control::MotorCANBase* motors[] = {motor1,motor2};
    float current = 0;
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            if (current == 0) {
                current = 200 * PI;
                flywheel1->SetSpeed(current);
                flywheel2->SetSpeed(current);
            } else {
                current = 0;
                flywheel1->SetSpeed(current);
                flywheel2->SetSpeed(current);
            }
        }
        flywheel1->CalcOutput();
        flywheel2->CalcOutput();
        control::MotorCANBase::TransmitOutput(motors, 2);
        flywheel1->PrintData();
        flywheel2->PrintData();
        osDelay(1);
    }
}
