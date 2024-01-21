/*###########################################################
# Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "protocol.h"




communication::Referee* referee = nullptr;
bsp::UART* referee_uart = nullptr;

communication::Referee* refereeRc = nullptr;
bsp::UART* refereeRc_uart = nullptr;



void RM_RTOS_Init(void) {
   print_use_usb();

   referee_uart = new bsp::UART(&huart1);
   referee_uart->SetupRx(300);
   referee_uart->SetupTx(300);

   refereeRc_uart = new bsp::UART(&huart6);
   refereeRc_uart->SetupRx(300);
   refereeRc_uart->SetupTx(300);

   referee = new communication::Referee(referee_uart);

   refereeRc = new communication::Referee(refereeRc_uart);
}

void RM_RTOS_Threads_Init(void) {

}

void RM_RTOS_Default_Task(const void* argument) {
   UNUSED(argument);

   while (true) {
       set_cursor(0, 0);
       clear_screen();
       //        print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
       //        print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current /
       //        1000.0); print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
       //        print("Chassis Power Limit: %d\r\n",
       //        referee->game_robot_status.chassis_power_limit); print("\r\n"); print("Shooter
       //        Cooling Heat: %hu\r\n",
       //              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
       //        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
       //        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
       //        print("\r\n");
       //        print("Current HP %d/%d\n", referee->game_robot_status.remain_HP,
       //              referee->game_robot_status.max_HP);
       //        print("Remain bullet %d\n", referee->bullet_remaining.bullet_remaining_num_17mm);
       remote::keyboard_t keyboard = referee->remote_control.keyboard;
       remote::mouse_t mouse = referee->remote_control.mouse;
       print(
           "W: %d A: %d S: %d D: %d\r\n"
           "Q: %d E: %d R: %d F: %d G: %d\r\n"
           "Z: %d X: %d C: %d V: %d B: %d\r\n"
           "SHIFT: %d CTRL: %d\r\n"
           "Mouse: x: %d y: %d z: %d l: %d r: %d\r\n",
           keyboard.bit.W, keyboard.bit.A, keyboard.bit.S, keyboard.bit.D, keyboard.bit.Q,
           keyboard.bit.E, keyboard.bit.R, keyboard.bit.F, keyboard.bit.G, keyboard.bit.Z,
           keyboard.bit.X, keyboard.bit.C, keyboard.bit.V, keyboard.bit.B, keyboard.bit.SHIFT,
           keyboard.bit.CTRL, mouse.x, mouse.y, mouse.z, mouse.l, mouse.r);
       osDelay(1);
   }
}