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

#include "bsp_thread.h"

namespace bsp{

    Thread::Thread(thread_init_t init) {
        func_ = init.func;
        args_ = init.args;
        attr_ = init.attr;
        thread_handle_ =NULL;
    }
    Thread::~Thread() {
        osThreadTerminate(thread_handle_);
    }
    void Thread::Start() {
        thread_handle_ = osThreadNew(func_, this, &attr_);
    }
    void Thread::Join() {
        osThreadJoin(thread_handle_);
    }
    void Thread::Pause() {
        osThreadSuspend(thread_handle_);
    }
    void Thread::Resume() {
        osThreadResume(thread_handle_);
    }
    void Thread::Wait(uint32_t millisec,uint32_t signal) {
        if(signal == 0){
            signal = rx_signal_;
        }
        if(millisec==0){
            osThreadFlagsWait(signal, osFlagsWaitAny, osWaitForever);
        }
        osThreadFlagsWait(signal, osFlagsWaitAny, millisec);
    }

    void Thread::Set(uint32_t signal) {
        if(signal == 0){
            signal = rx_signal_;
        }
        osThreadFlagsSet(thread_handle_, signal);
    }

    void* Thread::GetArgs() {
        return args_;
    }

    EventThread::EventThread(thread_init_t init) : Thread(init) {
        Start();
    }
    void EventThread::Start() {
        thread_handle_ = osThreadNew(ThreadFunc, this, &attr_);
    }
    void EventThread::ThreadFunc(void* args) {
        EventThread* instance = reinterpret_cast<EventThread*>(args);
        void* user_instance = reinterpret_cast<EventThread*>(instance)->GetArgs();
        while(true){
            osThreadFlagsWait(rx_signal_, osFlagsWaitAny, osWaitForever);
            instance->func_(user_instance);
        }
    }

}
