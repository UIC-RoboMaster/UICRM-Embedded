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

// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// Switch-case FSM performance baseline (for comparison)

#include <cstdlib>

#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"

enum class S1 { A, B, C };
enum class S2 { X, Y };
enum class S3 { IDLE, RUN };

enum class S_BIG { S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11 };

enum class S_HEAVY { A, B };

// ===== 状态变量 =====
static S1 s1 = S1::A;
static S2 s2 = S2::X;
static S3 s3 = S3::IDLE;
static S_BIG s_big = S_BIG::S0;
static S_HEAVY s_heavy = S_HEAVY::A;

uint32_t cpu_hz;

// ===== FSM STEP FUNCTIONS =====

inline void step_fsm1(uint8_t input) {
    switch (s1) {
        case S1::A:
            if (input > 100)
                s1 = S1::B;
            break;
        case S1::B:
            if (input > 200)
                s1 = S1::C;
            break;
        case S1::C:
            if (input < 50)
                s1 = S1::A;
            break;
    }
}

inline void step_fsm2(uint8_t fsm1_out) {
    switch (s2) {
        case S2::X:
            if (fsm1_out == (uint8_t)S1::C)
                s2 = S2::Y;
            break;
        case S2::Y:
            if (fsm1_out == (uint8_t)S1::A)
                s2 = S2::X;
            break;
    }
}

inline void step_fsm3(uint8_t fsm2_out) {
    switch (s3) {
        case S3::IDLE:
            if (fsm2_out == (uint8_t)S2::Y)
                s3 = S3::RUN;
            break;
        case S3::RUN:
            if (fsm2_out == (uint8_t)S2::X)
                s3 = S3::IDLE;
            break;
    }
}

inline void step_fsm_big(uint16_t input) {
    static int i = 0;

    switch (s_big) {
        case S_BIG::S0:
        case S_BIG::S1:
        case S_BIG::S2:
        case S_BIG::S3:
        case S_BIG::S4:
        case S_BIG::S5:
        case S_BIG::S6:
        case S_BIG::S7:
        case S_BIG::S8:
        case S_BIG::S9:
        case S_BIG::S10:
            i++;
            if (input > (i * 50)) {
                s_big = static_cast<S_BIG>((int)s_big + 1);
            }
            break;

        case S_BIG::S11:
            if (input < 10) {
                s_big = S_BIG::S0;
            }
            break;
    }
}

inline void step_fsm_heavy(uint16_t x) {
    switch (s_heavy) {
        case S_HEAVY::A: {
            bool cond = (x % 3 == 0) && (x % 5 != 0) && (x * x > 1000) && ((x ^ 0x55) & 0x0F) &&
                        (x > 123 && x < 300);

            if (cond)
                s_heavy = S_HEAVY::B;
            break;
        }
        case S_HEAVY::B:
            if (x < 50)
                s_heavy = S_HEAVY::A;
            break;
    }
}

// ===== 工具函数 =====

uint32_t cycleToNanoSec(uint32_t cyc) {
    return (uint64_t)cyc * 1000000000ULL / cpu_hz;
}

// ===== RTOS 初始化 =====

void RM_RTOS_Init(void) {
    print_use_uart(&huart6, true, 921600);
    cpu_hz = HAL_RCC_GetHCLKFreq();

    print("\r\nRunning SWITCH-CASE FSM...\r\n");
}

// ===== 主测试任务 =====

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    DWT_Init(cpu_hz);

    const int TEST_COUNT = 1000000;

    uint32_t cycle_time_sum = 0;
    uint32_t cycle_time_big[] = {0, 0, 0};
    uint32_t cycle_time_heavy[] = {0, 0, 0};

    for (int i = 0; i < TEST_COUNT; ++i) {
        uint8_t input = rand() % 1;
        uint16_t big_input = (i * 13) % 600;

        uint32_t cycles_start = DWT->CYCCNT;

        step_fsm1(input);
        step_fsm2((uint8_t)s1);
        step_fsm3((uint8_t)s2);

        cycle_time_big[0] = DWT->CYCCNT;
        step_fsm_big(big_input);
        cycle_time_big[1] = DWT->CYCCNT;

        cycle_time_heavy[0] = DWT->CYCCNT;
        step_fsm_heavy(big_input);
        cycle_time_heavy[1] = DWT->CYCCNT;

        uint32_t cycles_end = DWT->CYCCNT;

        cycle_time_sum += cycles_end - cycles_start;
        cycle_time_big[2] += cycle_time_big[1] - cycle_time_big[0];
        cycle_time_heavy[2] += cycle_time_heavy[1] - cycle_time_heavy[0];
    }

    print("CPU frequency: %lu\r\n", cpu_hz);
    print("Total run stage cycles: %lu\r\n", TEST_COUNT);

    print("Average cycles (ALL FSM): %lu (%lu ns)\r\n", cycle_time_sum / TEST_COUNT,
          cycleToNanoSec(cycle_time_sum / TEST_COUNT));

    print("Average BIG FSM cycles: %lu (%lu ns)\r\n", cycle_time_big[2] / TEST_COUNT,
          cycleToNanoSec(cycle_time_big[2] / TEST_COUNT));

    print("Average HEAVY FSM cycles: %lu (%lu ns)\r\n", cycle_time_heavy[2] / TEST_COUNT,
          cycleToNanoSec(cycle_time_heavy[2] / TEST_COUNT));
}
