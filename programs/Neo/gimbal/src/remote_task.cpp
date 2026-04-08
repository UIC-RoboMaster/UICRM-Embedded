/*###########################################################
# Copyright (c) 2026-2027. BNU-HKBU UIC RoboMaster         #
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

#include "Automata.h"
#include "remote_task.h"
#include "imu_task.h"
#include "minipc_task.h"

remote::DBUS* dbus = nullptr;

bool is_activate = true;
RemoteMode remote_mode = REMOTE_MODE_AUTOPILOT;
ShootFricMode shoot_fric_wheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_mode = SHOOT_MODE_STOP;
BulletCapMode bullet_cap_mode = BULLET_CAP_MODE_CLOSE;

RemoteMode last_remote_mode = REMOTE_MODE_SPIN;

void init_remote() {
    dbus = new remote::DBUS(&huart3);
}

osThreadId_t remoteTaskHandle;
void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);

    // bool is_dbus_offline = true;
    bool is_robot_dead = true;
    bool is_referee_shoot_available = false;

    auto activate_aut = control::AutomataBuilder<ActivateStates>()
        .item<control::AutomataInputRaw>(is_robot_dead)
        .transition<KILLED, ACTIVE>(TRANLOGIC {
            const auto& is_dead = COMPONENT(0);
            return dbus->IsOnline() && dbus->swr != remote::DOWN && !is_dead.get();
        })
        .transition<ACTIVE, KILLED>(TRANLOGIC {
            const auto& is_dead = COMPONENT(0);
            return !(dbus->IsOnline() && dbus->swr != remote::DOWN && !is_dead.get());
        })
        .build<KILLED>();

    auto tranlogic_next_remote_mode = TRANLOGIC {
        const auto& swr = COMPONENT(0);
        return swr.upEdge() && swr.get() == remote::UP;
    };
    auto remote_mode_aut = control::AutomataBuilder<RemoteMode>()
        .item<control::AutomataInputRemote>(static_cast<remote::switch_t>(dbus->swr))
        .transition<REMOTE_MODE_AUTOPILOT, REMOTE_MODE_FOLLOW>(tranlogic_next_remote_mode)
        .transition<REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN>(tranlogic_next_remote_mode)
        .transition<REMOTE_MODE_SPIN, REMOTE_MODE_AUTOPILOT>(tranlogic_next_remote_mode)
        .build<REMOTE_MODE_AUTOPILOT>();

    /*
    todo: should construct more stable logic.
          Example: SHOOT_FRIC_MODE_PREPARING should apply to prevent low-speed shoot.
    */
    auto tranlogic_fric_wheel_trigger = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        return swl.upEdge() && swl.get() == remote::UP;
    };
    auto fric_wheel_aut = control::AutomataBuilder<ShootFricMode>()
        .item<control::AutomataInputRemote>(static_cast<remote::switch_t>(dbus->swl))
        .transition<SHOOT_FRIC_MODE_STOP, SHOOT_FRIC_MODE_PREPARED>(tranlogic_fric_wheel_trigger)
        .transition<SHOOT_FRIC_MODE_PREPARED, SHOOT_FRIC_MODE_STOP>(tranlogic_fric_wheel_trigger)
        .build<SHOOT_FRIC_MODE_STOP>();

    // todo add single shoot
    auto shoot_aut = control::AutomataBuilder<ShootMode>()
        .item<control::AutomataInputRemote>(static_cast<remote::switch_t>(dbus->swl))
        .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
        .item<control::AutomataInputRaw>(is_referee_shoot_available)
        .transition<SHOOT_MODE_STOP, SHOOT_MODE_BURST>(TRANLOGIC {
            const auto& swl = COMPONENT(0);
            const auto& fric_state = COMPONENT(1);
            const auto& referee_permit = COMPONENT(2);
            return swl.get() == remote::DOWN &&
                fric_state.get() == SHOOT_FRIC_MODE_PREPARED &&
                referee_permit.get();
        })
        .transition<SHOOT_MODE_BURST, SHOOT_MODE_STOP>(TRANLOGIC {
            const auto& swl = COMPONENT(0);
            const auto& fric_state = COMPONENT(1);
            const auto& referee_permit = COMPONENT(2);
            return !(swl.get() == remote::DOWN &&
                fric_state.get() == SHOOT_FRIC_MODE_PREPARED &&
                referee_permit.get());
        })
        .build<SHOOT_MODE_STOP>();

    auto tranlogic_bullet_cap_trigger = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& fric_state = COMPONENT(1);
        return swl.downEdge() && swl.get() == remote::DOWN && fric_state.get() == SHOOT_FRIC_MODE_STOP;
    };
    auto bullet_cap_aut = control::AutomataBuilder<BulletCapMode>()
        .item<control::AutomataInputRemote>(static_cast<remote::switch_t>(dbus->swl))
        .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
        .transition<BULLET_CAP_MODE_CLOSE, BULLET_CAP_MODE_OPEN>(tranlogic_bullet_cap_trigger)
        .transition<BULLET_CAP_MODE_OPEN, BULLET_CAP_MODE_CLOSE>(tranlogic_bullet_cap_trigger)
        .build<BULLET_CAP_MODE_CLOSE>();

    while (true) {
        osDelay(REMOTE_OS_DELAY);

#ifdef HAS_REFEREE
        // Kill Detection
        is_robot_dead = referee->game_robot_status.remain_HP == 0;
        is_referee_shoot_available = (referee->game_robot_status.shooter_heat_limit -
                              referee->power_heat_data.shooter_id1_17mm_cooling_heat) >= 100 &&
                             // referee->bullet_remaining.bullet_remaining_num_17mm > 0 &&
                             imu->CaliDone();
#else
        is_robot_dead = false;
        is_referee_shoot_available = true;
#endif

        activate_aut.input(std::make_tuple(is_robot_dead));
        is_activate = activate_aut.state() == KILLED ? false : true;
        if (!is_activate) continue;

        remote_mode_aut.input(std::make_tuple(dbus->swr));
        remote_mode = remote_mode_aut.state();

        fric_wheel_aut.input(std::make_tuple(dbus->swl));
        shoot_fric_wheel_mode = fric_wheel_aut.state();

        shoot_aut.input(std::make_tuple(
            dbus->swl,
            shoot_fric_wheel_mode,
            is_referee_shoot_available));
        shoot_mode = shoot_aut.state();

        bullet_cap_aut.input(std::make_tuple(dbus->swl, shoot_fric_wheel_mode));
        bullet_cap_mode = bullet_cap_aut.state();

    }
}
