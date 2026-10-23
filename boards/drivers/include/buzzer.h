/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#pragma once

#include "bsp_pwm.h"

namespace driver {

    /**
     * @brief 蜂鸣器的音符
     * @note 一共有三个八度
     */
    /**
     * @brief buzzer notes
     * @note there are three octaves in total
     */
    enum class BuzzerNote {
        Do1L = 262,           // C4: 261.63 Hz, 3822 us
        Do1SharpL = 277,      // C#4: 277.18 Hz, 3608 us
        Re2L = 294,           // D4: 293.66 Hz, 3405 us
        Re2SharpL = 311,      // D#4: 311.13 Hz, 3214 us
        Mi3L = 330,           // E4: 329.63 Hz, 3034 us
        Fa4L = 349,           // F4: 349.23 Hz, 2863 us
        Fa4SharpL = 370,      // F#4: 369.99 Hz, 2703 us
        So5L = 392,           // G4: 392.00 Hz, 2551 us
        So5SharpL = 415,      // G#4: 415.30 Hz, 2408 us
        La6L = 440,           // A4: 440.00 Hz, 2273 us
        La6SharpL = 466,      // A#4: 466.16 Hz, 2145 us
        Si7L = 494,           // B4: 493.88 Hz, 2025 us

        Do1M = 523,           // C5: 523.25 Hz, 1911 us
        Do1SharpM = 554,      // C#5: 554.37 Hz, 1804 us
        Re2M = 587,           // D5: 587.33 Hz, 1703 us
        Re2SharpM = 622,      // D#5: 622.25 Hz, 1607 us
        Mi3M = 659,           // E5: 659.26 Hz, 1517 us
        Fa4M = 698,           // F5: 698.46 Hz, 1432 us
        Fa4SharpM = 740,      // F#5: 739.99 Hz, 1351 us
        So5M = 784,           // G5: 783.99 Hz, 1276 us
        So5SharpM = 831,      // G#5: 830.61 Hz, 1204 us
        La6M = 880,           // A5: 880.00 Hz, 1136 us
        La6SharpM = 932,      // A#5: 932.33 Hz, 1073 us
        Si7M = 988,           // B5: 987.77 Hz, 1012 us

        Do1H = 1047,          // C6: 1046.50 Hz, 956 us
        Do1SharpH = 1109,     // C#6: 1108.73 Hz, 902 us
        Re2H = 1175,          // D6: 1174.66 Hz, 851 us
        Re2SharpH = 1245,     // D#6: 1244.51 Hz, 804 us
        Mi3H = 1319,          // E6: 1318.51 Hz, 758 us
        Fa4H = 1397,          // F6: 1396.91 Hz, 716 us
        Fa4SharpH = 1480,     // F#6: 1479.98 Hz, 676 us
        So5H = 1568,          // G6: 1567.98 Hz, 638 us
        So5SharpH = 1661,     // G#6: 1661.22 Hz, 602 us
        La6H = 1760,          // A6: 1760.00 Hz, 568 us
        La6SharpH = 1865,     // A#6: 1864.66 Hz, 536 us
        Si7H = 1976,          // B6: 1975.53 Hz, 506 us

        Silent = 0,
        Finish = -1,
    };

    /**
     * @brief 延迟的音符
     * @note 用于播放音乐
     * @note 数组可以编曲
     */
    /**
     * @brief delayed buzzer notes
     * @note used for playing music
     * @note array can be used to compose music
     */
    struct BuzzerNoteDelayed {
        BuzzerNote note;
        uint32_t delay;
    };

    /**
     * @brief 蜂鸣器的延迟函数类型
     * @param milli 延迟的毫秒数
     */
    typedef void (*buzzer_delay_t)(uint32_t milli);

    /**
     * @brief 蜂鸣器
     * @note 用于播放音乐
     * @note 有三个八度哦
     */
    /**
     * @brief buzzer
     * @note used for playing music
     * @note there are three octaves
     */
    class Buzzer {
      public:
        /**
         * @brief 蜂鸣器实例构造函数
         * @param htim       hal定时器句柄
         * @param channel    定时器通道，可选[1, 2, 3, 4]
         * @param clock_freq 定时器时钟频率
         */
        /**
         * @brief constructor for a buzzer instance
         *
         * @param htim       hal timer handle
         * @param channel    timer channel associated with the timer choose from [1,
         * 2, 3, 4]
         * @param clock_freq clock frequency associated with the timer
         */
        Buzzer(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq);

        /**
         * @brief 唱一个音符
         * @param note 音符
         */
        /**
         * @brief sing a single tone indefinitely long
         *
         * @param note  note frequency to sing
         */
        void SingTone(const BuzzerNote& note);

        /**
         * @brief 唱一首曲子
         *
         * @param notes      延迟音符数组，可以编曲
         * @param delay_func 延迟函数，可以自定义
         *                  默认为HAL_Delay实现
         * @note 在RTOS内需要使用RTOS的延迟函数
         */
        /**
         * @brief sing a sequence of delayed notes
         *
         * @param notes       pointer to an array of delayed notes
         * @param delay_func  a void function that can delay arbitrary number of
         * milliseconds,
         *                    defaults to HAL_Delay implementation
         */
        void SingSong(
            const BuzzerNoteDelayed* notes, buzzer_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); }
        );

        /**
         * @brief 停止唱歌
         * @note 会停止当前的唱歌
         */
        /**
         * @brief stop singing
         * @note will stop the current singing
         */
        void Off();

      private:
        /* pwm instance associated with the buzzer */
        bsp::PWM pwm_;
    };

}  // namespace driver
