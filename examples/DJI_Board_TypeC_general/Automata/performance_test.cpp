#include "Automata.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"

using namespace control;

// ===== 状态定义 =====
enum class S1 { A, B, C };
enum class S2 { X, Y };
enum class S3 { IDLE, RUN };
enum class S_BIG { S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11 };
enum class S_HEAVY { A, B };

// Switch-case 裸机 FSM
static S1 sw_fsm1;
static S2 sw_fsm2;
static S3 sw_fsm3;
static S_BIG sw_fsm_big;
static S_HEAVY sw_fsm_heavy;

uint32_t cpu_hz;

// ===== 初始化 =====
void RM_RTOS_Init(void) {
    print_use_uart(&BOARD_UART2, true, 921600);
    cpu_hz = HAL_RCC_GetHCLKFreq();
    print("\r\nRunning...\r\n");
}

// ===== 周期函数 =====
uint32_t cycleToNanoSec(uint32_t cyc) {
    return (uint64_t)cyc * 1000000000ULL / cpu_hz;
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    DWT_Init(cpu_hz);

    uint8_t input;
    uint16_t big_input;

    uint32_t build_start = DWT->CYCCNT;
    // FSM1
    auto fsm1 =
        control::AutomataBuilder<S1>()
            .item<control::AutomataInputEdge>(input)
            .item<control::AutomataInputEdge>(1.1f)
            .transition<S1::A, S1::B>(TRANLOGIC { return ins.template get<0>().get() > 100; })
            .transition<S1::B, S1::C>(TRANLOGIC { return ins.template get<0>().get() > 200; })
            .transition<S1::C, S1::A>(TRANLOGIC { return ins.template get<0>().get() < 50; })
            .build<S1::A>();

    // FSM2
    auto fsm2 =
        control::AutomataBuilder<S2>()
            .item<control::AutomataInputRaw>(S1::A)
            .transition<S2::X, S2::Y>(TRANLOGIC { return ins.template get<0>().get() == S1::C; })
            .transition<S2::Y, S2::X>(TRANLOGIC { return ins.template get<0>().get() == S1::A; })
            .build<S2::X>();

    // FSM3
    auto fsm3 = control::AutomataBuilder<S3>()
                    .item<control::AutomataInputRaw>(S2::X)
                    .transition<S3::IDLE, S3::RUN>(
                        TRANLOGIC { return ins.template get<0>().get() == S2::Y; })
                    .transition<S3::RUN, S3::IDLE>(
                        TRANLOGIC { return ins.template get<0>().get() == S2::X; })
                    .build<S3::IDLE>();

    // Big FSM
    auto fsm_big = control::AutomataBuilder<S_BIG>()
                       .item<control::AutomataInputEdge>(uint16_t{})

                       .transition<S_BIG::S0, S_BIG::S1>(
                           TRANLOGIC { return ins.template get<0>().get() > 50; })

                       .transition<S_BIG::S1, S_BIG::S2>(
                           TRANLOGIC { return ins.template get<0>().get() > 100; })

                       .transition<S_BIG::S2, S_BIG::S3>(
                           TRANLOGIC { return ins.template get<0>().get() > 150; })

                       .transition<S_BIG::S3, S_BIG::S4>(
                           TRANLOGIC { return ins.template get<0>().get() > 200; })

                       .transition<S_BIG::S4, S_BIG::S5>(
                           TRANLOGIC { return ins.template get<0>().get() > 250; })

                       .transition<S_BIG::S5, S_BIG::S6>(
                           TRANLOGIC { return ins.template get<0>().get() > 300; })

                       .transition<S_BIG::S6, S_BIG::S7>(
                           TRANLOGIC { return ins.template get<0>().get() > 350; })

                       .transition<S_BIG::S7, S_BIG::S8>(
                           TRANLOGIC { return ins.template get<0>().get() > 400; })

                       .transition<S_BIG::S8, S_BIG::S9>(
                           TRANLOGIC { return ins.template get<0>().get() > 450; })

                       .transition<S_BIG::S9, S_BIG::S10>(
                           TRANLOGIC { return ins.template get<0>().get() > 500; })

                       .transition<S_BIG::S10, S_BIG::S11>(
                           TRANLOGIC { return ins.template get<0>().get() > 550; })

                       .transition<S_BIG::S11, S_BIG::S0>(
                           TRANLOGIC { return ins.template get<0>().get() < 10; })

                       .build<S_BIG::S0>();

    // Heavy FSM
    auto fsm_heavy = control::AutomataBuilder<S_HEAVY>()
                         .item<control::AutomataInputEdge>(big_input)
                         .transition<S_HEAVY::A, S_HEAVY::B>(TRANLOGIC {
                             auto x = ins.template get<0>().get();
                             return (x % 3 == 0) && (x % 5 != 0) && (x * x > 1000) &&
                                    ((x ^ 0x55) & 0x0F) && (x > 123 && x < 300);
                         })
                         .transition<S_HEAVY::B, S_HEAVY::A>(
                             TRANLOGIC { return ins.template get<0>().get() < 50; })
                         .build<S_HEAVY::A>();
    uint32_t build_end = DWT->CYCCNT;

    const int TEST_COUNT = 1000000;
    uint32_t cycle_sum = 0, cycle_1[3] = {0}, cycle_2[3] = {0}, cycle_3[3] = {0},
             cycle_big[3] = {0, 0, 0}, cycle_heavy[3] = {0, 0, 0}, cycle_switch = 0;

    for (int i = 0; i < TEST_COUNT; i++) {
        input = rand() % 1;
        big_input = (i * 13) % 600;

        // FSM 自动机
        uint32_t t0 = DWT->CYCCNT;
        cycle_1[0] = DWT->CYCCNT;
        fsm1.input(std::make_tuple(input, 1.0));
        cycle_1[1] = DWT->CYCCNT;
        cycle_2[0] = DWT->CYCCNT;
        fsm2.input(std::make_tuple(fsm1.state()));
        cycle_2[1] = DWT->CYCCNT;
        cycle_3[0] = DWT->CYCCNT;
        fsm3.input(std::make_tuple(fsm2.state()));
        cycle_3[1] = DWT->CYCCNT;
        cycle_big[0] = DWT->CYCCNT;
        fsm_big.input(std::make_tuple(big_input));
        cycle_big[1] = DWT->CYCCNT;
        cycle_heavy[0] = DWT->CYCCNT;
        fsm_heavy.input(std::make_tuple(big_input));
        cycle_heavy[1] = DWT->CYCCNT;
        uint32_t t1 = DWT->CYCCNT;
        cycle_sum += t1 - t0;
        cycle_1[2] += cycle_1[1] - cycle_1[0];
        cycle_2[2] += cycle_2[1] - cycle_2[0];
        cycle_3[2] += cycle_3[1] - cycle_3[0];
        cycle_big[2] += cycle_big[1] - cycle_big[0];
        cycle_heavy[2] += cycle_heavy[1] - cycle_heavy[0];

        // Switch-case
        uint32_t ts = DWT->CYCCNT;
        // FSM1
        switch (sw_fsm1) {
            case S1::A:
                if (input > 100)
                    sw_fsm1 = S1::B;
                break;
            case S1::B:
                if (input > 200)
                    sw_fsm1 = S1::C;
                break;
            case S1::C:
                if (input < 50)
                    sw_fsm1 = S1::A;
                break;
        }
        // FSM2
        switch (sw_fsm2) {
            case S2::X:
                if (sw_fsm1 == S1::C)
                    sw_fsm2 = S2::Y;
                break;
            case S2::Y:
                if (sw_fsm1 == S1::A)
                    sw_fsm2 = S2::X;
                break;
        }
        // FSM3
        switch (sw_fsm3) {
            case S3::IDLE:
                if (sw_fsm2 == S2::Y)
                    sw_fsm3 = S3::RUN;
                break;
            case S3::RUN:
                if (sw_fsm2 == S2::X)
                    sw_fsm3 = S3::IDLE;
                break;
        }
        // Big FSM
        static int j = 0;
        switch (sw_fsm_big) {
            case S_BIG::S0:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S1;
                break;
            case S_BIG::S1:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S2;
                break;
            case S_BIG::S2:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S3;
                break;
            case S_BIG::S3:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S4;
                break;
            case S_BIG::S4:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S5;
                break;
            case S_BIG::S5:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S6;
                break;
            case S_BIG::S6:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S7;
                break;
            case S_BIG::S7:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S8;
                break;
            case S_BIG::S8:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S9;
                break;
            case S_BIG::S9:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S10;
                break;
            case S_BIG::S10:
                if (big_input > (++j * 50))
                    sw_fsm_big = S_BIG::S11;
                break;
            case S_BIG::S11:
                if (big_input < 10)
                    sw_fsm_big = S_BIG::S0;
                break;
        }
        // Heavy FSM
        switch (sw_fsm_heavy) {
            case S_HEAVY::A: {
                bool cond = (big_input % 3 == 0) && (big_input % 5 != 0) &&
                            (big_input * big_input > 1000) && ((big_input ^ 0x55) & 0x0F) &&
                            (big_input > 123 && big_input < 300);
                if (cond)
                    sw_fsm_heavy = S_HEAVY::B;
            } break;
            case S_HEAVY::B:
                if (big_input < 50)
                    sw_fsm_heavy = S_HEAVY::A;
                break;
        }
        uint32_t te = DWT->CYCCNT;
        cycle_switch += te - ts;
    }
    uint32_t fsm_cycle_sum = cycle_1[2] / TEST_COUNT + cycle_2[2] / TEST_COUNT +
                             cycle_3[2] / TEST_COUNT + cycle_big[2] / TEST_COUNT +
                             cycle_heavy[2] / TEST_COUNT;

    print("CPU frequency: %lu\r\n", cpu_hz);
    print("Build cycles: %lu(%lu ns)\r\n", build_end - build_start,
          cycleToNanoSec(build_end - build_start));
    print("\r\n");
    print("Common FSM1 avg cycles: %lu(%lu ns)\r\n", cycle_1[2] / TEST_COUNT,
          cycleToNanoSec(cycle_1[2] / TEST_COUNT));
    print("Common FSM2 avg cycles: %lu(%lu ns)\r\n", cycle_2[2] / TEST_COUNT,
          cycleToNanoSec(cycle_2[2] / TEST_COUNT));
    print("Common FSM3 avg cycles: %lu(%lu ns)\r\n", cycle_3[2] / TEST_COUNT,
          cycleToNanoSec(cycle_3[2] / TEST_COUNT));
    print("Big FSM avg cycles: %lu(%lu ns)\r\n", cycle_big[2] / TEST_COUNT,
          cycleToNanoSec(cycle_big[2] / TEST_COUNT));
    print("Heavy FSM avg cycles: %lu(%lu ns)\r\n", cycle_heavy[2] / TEST_COUNT,
          cycleToNanoSec(cycle_heavy[2] / TEST_COUNT));
    print("\r\n");
    print("5 FSMs avg cycles: %lu(%lu ns)\r\n", fsm_cycle_sum, cycleToNanoSec(fsm_cycle_sum));
    print("5 Switch-cases avg cycles: %lu(%lu ns)\r\n", cycle_switch / TEST_COUNT,
          cycleToNanoSec(cycle_switch / TEST_COUNT));
    print("\r\n");
    print("1 FSM avg cycles: %lu(%lu ns)\r\n", fsm_cycle_sum / 5,
          cycleToNanoSec(fsm_cycle_sum / 5));
    print("1 Switch-case avg cycles: %lu(%lu ns)\r\n", cycle_switch / 5 / TEST_COUNT,
          cycleToNanoSec(cycle_switch / 5 / TEST_COUNT));
}