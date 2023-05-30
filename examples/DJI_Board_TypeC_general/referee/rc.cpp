#include "main.h"

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "dbus.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

class RefereeUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final {
        osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL);
    }
};

communication::Referee* referee = nullptr;
RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
    UNUSED(arg);
    uint32_t length;
    uint8_t* data;

    while (true) {
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
            length = referee_uart->Read(&data);
            referee->Receive(communication::package_t{data, (int)length});
        }
    }
}

const osThreadAttr_t refereeRcTaskAttribute = {.name = "refereercTask",
                                               .attr_bits = osThreadDetached,
                                               .cb_mem = nullptr,
                                               .cb_size = 0,
                                               .stack_mem = nullptr,
                                               .stack_size = 128 * 4,
                                               .priority = (osPriority_t)osPriorityNormal,
                                               .tz_module = 0,
                                               .reserved = 0};
osThreadId_t refereeRcTaskHandle;

class RefereeRcUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final {
        osThreadFlagsSet(refereeRcTaskHandle, RX_SIGNAL);
    }
};

communication::Referee* refereeRc = nullptr;
RefereeRcUART* refereeRc_uart = nullptr;

void refereeRcTask(void* arg) {
    UNUSED(arg);
    uint32_t length;
    uint8_t* data;

    while (true) {
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
            length = refereeRc_uart->Read(&data);
            referee->Receive(communication::package_t{data, (int)length});
        }
    }
}

void RM_RTOS_Init(void) {
    print_use_usb();

    referee_uart = new RefereeUART(&huart6);
    referee_uart->SetupRx(300);
    referee_uart->SetupTx(300);

    refereeRc_uart = new RefereeRcUART(&huart1);
    refereeRc_uart->SetupRx(300);
    refereeRc_uart->SetupTx(300);

    referee = new communication::Referee;

    refereeRc = new communication::Referee;
}

void RM_RTOS_Threads_Init(void) {
    refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
    refereeRcTaskHandle = osThreadNew(refereeRcTask, nullptr, &refereeRcTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        //        print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
        //        print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
        //        print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
        //        print("Chassis Power Limit: %d\r\n", referee->game_robot_status.chassis_power_limit);
        //        print("\r\n");
        //        print("Shooter Cooling Heat: %hu\r\n",
        //              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        //        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        //        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        //        print("\r\n");
        //        print("Current HP %d/%d\n", referee->game_robot_status.remain_HP,
        //              referee->game_robot_status.max_HP);
        //        print("Remain bullet %d\n", referee->bullet_remaining.bullet_remaining_num_17mm);
        remote::keyboard_t keyboard = referee->remote_control.keyboard;
        remote::mouse_t mouse = referee->remote_control.mouse;
        print("W: %d A: %d S: %d D: %d\r\n"
            "Q: %d E: %d R: %d F: %d G: %d\r\n"
            "Z: %d X: %d C: %d V: %d B: %d\r\n"
            "SHIFT: %d CTRL: %d\r\n"
            "Mouse: x: %d y: %d z: %d l: %d r: %d\r\n",
            keyboard.bit.W, keyboard.bit.A, keyboard.bit.S, keyboard.bit.D,
            keyboard.bit.Q, keyboard.bit.E, keyboard.bit.R, keyboard.bit.F, keyboard.bit.G,
            keyboard.bit.Z, keyboard.bit.X, keyboard.bit.C, keyboard.bit.V, keyboard.bit.B,
            keyboard.bit.SHIFT, keyboard.bit.CTRL,
            mouse.x, mouse.y, mouse.z, mouse.l, mouse.r);
        osDelay(1);
    }
}