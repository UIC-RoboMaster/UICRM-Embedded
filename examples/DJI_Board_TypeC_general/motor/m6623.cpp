#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart6);

    can1 = new bsp::CAN(&hcan1, 0x201);
    motor = new control::Motor6623(can1, 0x205);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    control::MotorCANBase* motors[] = {motor};

    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    while (true) {
        motor->PrintData();
        if (key.Read())
            motor->SetOutput(3000);
        else
            motor->SetOutput(0);
        control::MotorCANBase::TransmitOutput(motors, 1);
        motor->PrintData();
        osDelay(100);
    }
}