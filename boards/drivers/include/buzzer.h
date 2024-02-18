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
        Do1L = 262,  ///*261.63Hz*/    3822us
        Re2L = 294,  ///*293.66Hz*/    3405us
        Mi3L = 330,  ///*329.63Hz*/    3034us
        Fa4L = 349,  ///*349.23Hz*/    2863us
        So5L = 392,  ///*392.00Hz*/    2551us
        La6L = 440,  ///*440.00Hz*/    2272us
        Si7L = 494,  ///*493.88Hz*/    2052us

        Do1M = 523,  ///*523.25Hz*/    1911us
        Re2M = 587,  ///*587.33Hz*/    1703us
        Mi3M = 659,  ///*659.26Hz*/    1517us
        Fa4M = 698,  ///*698.46Hz*/    1432us
        So5M = 784,  ///*784.00Hz*/    1276us
        La6M = 880,  ///*880.00Hz*/    1136us
        Si7M = 988,  ///*987.77Hz*/    1012us

        Do1H = 1047,  ///*1046.50Hz*/   956us
        Re2H = 1175,  ///*1174.66Hz*/   851us
        Mi3H = 1319,  ///*1318.51Hz*/   758us
        Fa4H = 1397,  ///*1396.91Hz*/   716us
        So5H = 1568,  ///*1567.98Hz*/   638us
        La6H = 1760,  ///*1760.00Hz*/   568us
        Si7H = 1976,  ///*1975.53Hz*/   506us

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
            const BuzzerNoteDelayed* notes,
            buzzer_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

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
