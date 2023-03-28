#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

bsp::CAN* can1 = NULL;
control::MotorCANBase* motor1 = NULL;
// control::MotorCANBase* motor2 = NULL;

void RM_RTOS_Init() {
    print_use_uart(&huart1);

    can1 = new bsp::CAN(&hcan1, 0x205, true);
    motor1 = new control::Motor6020(can1, 0x205);
    // motor2 = new control::Motor6020(can2, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    control::MotorCANBase* motors[] = {motor1};

    bsp::GPIO key(KEY_GPIO_Port, KEY_Pin);
    control::ConstrainedPID pid(8000, 5000, 10000, 500, 10000);
    bool last = 1;
    int count = 0;
    bool target = 0;
    while (true) {
        if (key.Read() && last == 0) {
            target = !target;
        }
        last = key.Read();
        float error = motor1->GetThetaDelta(target ? 0 : PI);
        // uint16_t output=pid.ComputeConstrainedOutput(abs(error)>0.01?error:0);
        uint16_t output = pid.ComputeConstrainedOutput(error);
        motor1->SetOutput(output);

        control::MotorCANBase::TransmitOutput(motors, 1);
        // set_cursor(0, 0);
        // clear_screen();
        count++;
        if (count == 1)
            print("%f,%f,%d,%d\n", motor1->GetTheta(), motor1->GetOmega(), output,
                  motor1->GetCurr()),
                count = 0;
        osDelay(10);
    }
}
