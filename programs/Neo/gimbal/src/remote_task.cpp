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
ShootMode last_shoot_mode = SHOOT_MODE_STOP;

void init_remote() {
    dbus = new remote::DBUS(&huart3);
}

osThreadId_t remoteTaskHandle;
void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);

    bool is_referee_robot_dead = true;
    bool is_referee_shoot_available = false;

    // 使/失能（上/下电）
    auto tranlogic_active_condition = TRANLOGIC {
        const auto& referee_cmd_dead = COMPONENT(0);
        return dbus->IsOnline() && dbus->swr != remote::DOWN && !referee_cmd_dead.get();
    };
    auto activate_aut = control::AutomataBuilder<ActivateStates>()
        .item<control::AutomataInputRaw>(is_referee_robot_dead)
        .transition<KILLED, ACTIVE>(tranlogic_active_condition)
        .transition<ACTIVE, KILLED>(tranlogic_active_condition, control::ReverseTag{})
        .build<KILLED>();

    // 控制模式
    auto tranlogic_next_remote_mode_trigger = TRANLOGIC {
        const auto& swr = COMPONENT(0);
        const auto& mode_change_key = COMPONENT(1);
        return (swr.downEdge() && swr.get() == remote::UP) || mode_change_key.upEdge();
    };
    auto remote_mode_aut = control::AutomataBuilder<RemoteMode>()
        .item<control::AutomataInputEdge>(dbus->swr)
        .item<control::AutomataInputEdge>(bool{}) // mode_change_key
        .transition<REMOTE_MODE_AUTOPILOT, REMOTE_MODE_FOLLOW>(tranlogic_next_remote_mode_trigger)
        .transition<REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN>(tranlogic_next_remote_mode_trigger)
        .transition<REMOTE_MODE_SPIN, REMOTE_MODE_AUTOPILOT>(tranlogic_next_remote_mode_trigger)
        .build<REMOTE_MODE_AUTOPILOT>();

    /*
    todo: should construct more stable logic.
          Example: SHOOT_FRIC_MODE_PREPARING should apply to prevent low-speed shoot.
    */
    // 摩擦轮
    auto tranlogic_fric_wheel_trigger = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& fric_key = COMPONENT(1);
        return (swl.downEdge() && swl.get() == remote::UP) || fric_key.upEdge();
    };
    auto fric_wheel_aut = control::AutomataBuilder<ShootFricMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputEdge>(bool{}) // friction_key
        .transition<SHOOT_FRIC_MODE_PREPARING, SHOOT_FRIC_MODE_STOP>(tranlogic_fric_wheel_trigger)
        .transition<SHOOT_FRIC_MODE_PREPARED, SHOOT_FRIC_MODE_STOP>(tranlogic_fric_wheel_trigger)
        .transition<SHOOT_FRIC_MODE_STOP, SHOOT_FRIC_MODE_PREPARING>(tranlogic_fric_wheel_trigger)
        .transition<SHOOT_FRIC_MODE_PREPARING, SHOOT_FRIC_MODE_PREPARED>(TRANLOGIC {
            UNUSED(ins);
            return true;
        })
        .build<SHOOT_FRIC_MODE_STOP>();

    // 射击（供弹轮）
    auto tranlogic_shooting_condition = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& fric_state = COMPONENT(1);
        const auto& referee_permit = COMPONENT(2);
        const auto& shoot_key = COMPONENT(3);
        const auto& remote_mode = COMPONENT(4);
        return (swl.get() == remote::DOWN || shoot_key.get()) &&
            fric_state.get() == SHOOT_FRIC_MODE_PREPARED &&
            referee_permit.get() &&
            (remote_mode.get() == REMOTE_MODE_AUTOPILOT ? minipc->target_angle.shoot_cmd : true);
    };
    auto shoot_aut = control::AutomataBuilder<ShootMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
        .item<control::AutomataInputRaw>(is_referee_shoot_available)
        .item<control::AutomataInputEdge>(bool{}) // shoot_key
        .item<control::AutomataInputRaw>(REMOTE_MODE_AUTOPILOT)
        .transition<SHOOT_MODE_SINGLE, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
        .transition<SHOOT_MODE_BURST, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
        .transition<SHOOT_MODE_STOP, SHOOT_MODE_SINGLE>(tranlogic_shooting_condition)
        .transition<SHOOT_MODE_SINGLE, SHOOT_MODE_BURST>(TRANLOGIC {
            const auto& swl = COMPONENT(0);
            const auto& shoot_key = COMPONENT(3);
            constexpr uint16_t burst_threshold_cycle = 300;
            return swl.lastUpdate() > burst_threshold_cycle ||
                shoot_key.lastUpdate() > burst_threshold_cycle;
        })
        .build<SHOOT_MODE_STOP>();

    // 子弹盖
    auto tranlogic_bullet_cap_trigger = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& fric_state = COMPONENT(1);
        return swl.downEdge() && swl.get() == remote::DOWN && fric_state.get() == SHOOT_FRIC_MODE_STOP;
    };
    auto bullet_cap_aut = control::AutomataBuilder<BulletCapMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
        .transition<BULLET_CAP_MODE_CLOSE, BULLET_CAP_MODE_OPEN>(tranlogic_bullet_cap_trigger)
        .transition<BULLET_CAP_MODE_OPEN, BULLET_CAP_MODE_CLOSE>(tranlogic_bullet_cap_trigger)
        .build<BULLET_CAP_MODE_CLOSE>();

    while (true) {
        osDelay(REMOTE_OS_DELAY);

#ifdef HAS_REFEREE
        // Kill Detection
        is_referee_robot_dead = referee->game_robot_status.remain_HP == 0;
        is_referee_shoot_available = (referee->game_robot_status.shooter_heat_limit -
                              referee->power_heat_data.shooter_id1_17mm_cooling_heat) >= 100 &&
                             // referee->bullet_remaining.bullet_remaining_num_17mm > 0 &&
                             imu->CaliDone();
#else
        is_referee_robot_dead = false;
        is_referee_shoot_available = true;
#endif

        activate_aut.input(std::make_tuple(is_referee_robot_dead));
        is_activate = activate_aut.state() == ACTIVE;
        if (!is_activate || !imu->DataReady() || !imu->CaliDone()) continue;

        remote_mode_aut.input(std::make_tuple(
            dbus->swr,
            static_cast<bool>(dbus->keyboard.bit.SHIFT)));
        last_remote_mode = remote_mode;
        remote_mode = remote_mode_aut.state();

        fric_wheel_aut.input(std::make_tuple(
            dbus->swl,
            static_cast<bool>(dbus->keyboard.bit.Z)));
        shoot_fric_wheel_mode = fric_wheel_aut.state();

        shoot_aut.input(std::make_tuple(
            dbus->swl,
            shoot_fric_wheel_mode,
            is_referee_shoot_available,
            dbus->mouse.l,
            remote_mode));
        last_shoot_mode = shoot_mode;
        shoot_mode = shoot_aut.state();

        bullet_cap_aut.input(std::make_tuple(dbus->swl, shoot_fric_wheel_mode));
        bullet_cap_mode = bullet_cap_aut.state();

    }
}
