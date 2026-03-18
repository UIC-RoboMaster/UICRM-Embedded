// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by gttgf on 2026/3/18.
//
#include "bsp_dwt.h"

#include "cmsis_os.h"
#include "bsp_print.h"
#include "bsp_gpio.h"
#include "Automata.h"

using namespace control;
using namespace communication;

enum class S1 { A, B, C };
enum class S2 { X, Y };
enum class S3 { IDLE, RUN };

enum class S_BIG {
    S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11
};

enum class S_HEAVY { A, B };

static Automata<S1>* fsm1;
static Automata<S2>* fsm2;
static Automata<S3>* fsm3;
static Automata<S_BIG>* fsm_big;
static Automata<S_HEAVY>* fsm_heavy;

uint32_t cpu_hz;

void RM_RTOS_Init(void) {
    print_use_uart(&huart6, true, 921600);
    cpu_hz = HAL_RCC_GetHCLKFreq();

    // set_cursor(0, 0);
    // clear_screen();
    print("\r\nRunning...\r\n");
}

void build_fsms() {
    // ===== FSM1 =====
    auto* b1 = new AutomataBuilder<S1>();
    (*b1)
        .input<remote::AutomataInputRemote>(uint8_t{}, "in1")
        .transition(S1::A, S1::B, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() > 100;
        })
        .transition(S1::B, S1::C, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() > 200;
        })
        .transition(S1::C, S1::A, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() < 50;
        });

    fsm1 = b1->build(S1::A);
    delete b1;

    // ===== FSM2 =====
    auto* b2 = new AutomataBuilder<S2>();
    (*b2)
        .input<remote::AutomataInputRemote>(uint8_t{}, "fsm1_out")
        .transition(S2::X, S2::Y, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() == (uint8_t)S1::C;
        })
        .transition(S2::Y, S2::X, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() == (uint8_t)S1::A;
        });

    fsm2 = b2->build(S2::X);
    delete b2;

    // ===== FSM3 =====
    auto* b3 = new AutomataBuilder<S3>();
    (*b3)
        .input<remote::AutomataInputRemote>(uint8_t{}, "fsm2_out")
        .transition(S3::IDLE, S3::RUN, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() == (uint8_t)S2::Y;
        })
        .transition(S3::RUN, S3::IDLE, TRANLOGIC {
            auto v = ins.get<remote::AutomataInputRemote>(uint8_t{}, 0);
            return v.get() == (uint8_t)S2::X;
        });

    fsm3 = b3->build(S3::IDLE);
    delete b3;

    // big fsm
    auto* b4 = new AutomataBuilder<S_BIG>();
    b4->input<remote::AutomataInputRemote>(uint16_t{}, "big_in");

    for (int i = 0; i < 11; ++i) {
        b4->transition(
            static_cast<S_BIG>(i),
            static_cast<S_BIG>(i + 1),
            TRANLOGIC {
                static int i;
                i++;
                auto v = ins.get<remote::AutomataInputRemote>(uint16_t{}, 0);
                return v.get() > (i * 50);
            }
        );
    }

    b4->transition(S_BIG::S11, S_BIG::S0, TRANLOGIC {
        auto v = ins.get<remote::AutomataInputRemote>(uint16_t{}, 0);
        return v.get() < 10;
    });

    fsm_big = b4->build(S_BIG::S0);
    delete b4;

    auto* b5 = new AutomataBuilder<S_HEAVY>();
    b5->input<remote::AutomataInputRemote>(uint16_t{}, "heavy");

    b5->transition(S_HEAVY::A, S_HEAVY::B, TRANLOGIC {
        auto v = ins.get<remote::AutomataInputRemote>(uint16_t{}, 0);
        uint16_t x = v.get();

        bool cond =
            (x % 3 == 0) &&
            (x % 5 != 0) &&
            (x * x > 1000) &&
            ((x ^ 0x55) & 0x0F) &&
            (x > 123 && x < 300);

        return cond;
    });

    b5->transition(S_HEAVY::B, S_HEAVY::A, TRANLOGIC {
        auto v = ins.get<remote::AutomataInputRemote>(uint16_t{}, 0);
        return v.get() < 50;
    });

    fsm_heavy = b5->build(S_HEAVY::A);
    delete b5;
}

uint32_t cycleToNanoSec(uint32_t cyc) {return (uint64_t)cyc * 1000000000ULL / cpu_hz;}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    DWT_Init(cpu_hz);

    uint32_t build_start = DWT->CYCCNT;
    build_fsms();
    uint32_t build_end = DWT->CYCCNT;

    const int TEST_COUNT = 1000000;

    uint32_t cycle_time_sum = 0;
    uint32_t cycle_time_big[] = {0, 0, 0};
    uint32_t cycle_time_heavy[] = {0, 0, 0};

    for (int i = 0; i < TEST_COUNT; ++i) {
        uint8_t input = rand() % 1;
        uint16_t big_input = (i * 13) % 600;

        uint32_t cycles_start = DWT->CYCCNT;
        fsm1->input(std::make_tuple(input));
        fsm2->input(std::make_tuple((uint8_t)fsm1->state()));
        fsm3->input(std::make_tuple((uint8_t)fsm2->state()));
        cycle_time_big[0]=DWT->CYCCNT;
        fsm_big->input(std::make_tuple(big_input));
        cycle_time_big[1]=DWT->CYCCNT;
        cycle_time_heavy[0]=DWT->CYCCNT;
        fsm_heavy->input(std::make_tuple(big_input));
        cycle_time_heavy[1]=DWT->CYCCNT;
        uint32_t cycles_end = DWT->CYCCNT;

        cycle_time_sum += cycles_end - cycles_start;
        cycle_time_big[2] += cycle_time_big[1] - cycle_time_big[0];
        cycle_time_heavy[2] += cycle_time_heavy[1] - cycle_time_heavy[0];
    }

    print("CPU frequency: %lu\r\n", cpu_hz);
    print("Build cycles: %lu(%lu ns)\r\n", build_end - build_start, cycleToNanoSec(build_end - build_start));
    print("Total run stage cycles: %lu\r\n", TEST_COUNT);
    print("Average cycles that all FSM step cycles: %lu(%lu ns)\r\n", cycle_time_sum / TEST_COUNT, cycleToNanoSec(cycle_time_sum / TEST_COUNT));
    print("Average big fsm step cycles: %lu(%lu ns)\r\n", cycle_time_big[2] / TEST_COUNT, cycleToNanoSec(cycle_time_big[2] / TEST_COUNT));
    print("Average heavy step fsm cycles: %lu(%lu ns)\r\n", cycle_time_heavy[2] / TEST_COUNT, cycleToNanoSec(cycle_time_heavy[2] / TEST_COUNT));
}