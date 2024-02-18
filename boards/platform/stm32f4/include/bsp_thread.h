/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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
#include "main.h"
#include "cmsis_os2.h"

namespace bsp {


    typedef void (*thread_func_t)(void* args);

    typedef struct {
        thread_func_t func;
        void* args;
        osThreadAttr_t attr;
    }thread_init_t;

    class Thread {
      public:
        Thread(thread_init_t init);

        virtual ~Thread();

        virtual void Start();

        void Join();

        void Pause();

        void Resume();

        void Wait(uint32_t millisec=0, uint32_t signal = 0);

        void Set(uint32_t signal = 0);

        void* GetArgs();

      protected:
        osThreadId_t thread_handle_;
        thread_func_t func_;
        osThreadAttr_t attr_;
        void* args_;
        static const uint32_t rx_signal_ = 1 << 0;
    };

    class EventThread: public Thread {
      public:
        EventThread(thread_init_t init);

        void Start() override final;
      private:
        static void ThreadFunc(void* args);
    };

}