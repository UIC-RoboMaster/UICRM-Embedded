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
#ifdef HAS_MINIPC
#include "minipc_task.h"
#endif

remote::DBUS* dbus = nullptr;

bool is_activate = false;
bool turbo_shoot = false;
#ifdef HAS_MINIPC
constexpr RemoteMode initial_remote_mode = REMOTE_MODE_AUTOPILOT;
#else
constexpr RemoteMode initial_remote_mode = REMOTE_MODE_FOLLOW;
#endif
RemoteMode remote_mode = initial_remote_mode;
ShootFricMode shoot_fric_wheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_mode = SHOOT_MODE_STOP;

RemoteMode last_remote_mode = REMOTE_MODE_SPIN;
ShootMode last_shoot_mode = SHOOT_MODE_STOP;

void init_dbus() {
    dbus = new remote::DBUS(&huart3);
}

void init_remote() {
    init_dbus();
}

osThreadId_t remoteTaskHandle;
void remoteTask(void* arg) {
    UNUSED(arg);
    osDelay(1000);

    bool is_controller_online = false;
    bool is_referee_robot_dead = true;
    bool is_referee_shoot_available = false;
    BoolEdgeDetector turbo_edge{false};

    // Automata definitions
    // clang-format off

    // 使/失能（上/下电）
    auto tranlogic_active_condition = TRANLOGIC {
        const auto& referee_cmd_dead = COMPONENT(0);
        const auto& controller_online = COMPONENT(1);
        return controller_online.get() && (!dbus->IsOnline() || dbus->swr != remote::DOWN) &&
            !referee_cmd_dead.get();
    };
    auto activate_aut = control::AutomataBuilder<ActivateStates>()
        .item<control::AutomataInputRaw>(is_referee_robot_dead)
        .item<control::AutomataInputRaw>(is_controller_online)
        .transition<KILLED, ACTIVE>(tranlogic_active_condition)
        .transition<ACTIVE, KILLED>(tranlogic_active_condition, control::ReverseTag{})
        .build<KILLED>();

    // 控制模式
    auto tranlogic_next_remote_mode_trigger = TRANLOGIC {
        const auto& swr = COMPONENT(0);
        const auto& mode_change_key = COMPONENT(1);
        const auto& vt13_change_button = COMPONENT(2);
        return (swr.downEdge() && swr.get() == remote::UP) || mode_change_key.upEdge() ||
            vt13_change_button.upEdge();
    };
    auto remote_mode_aut = control::AutomataBuilder<RemoteMode>()
        .item<control::AutomataInputEdge>(dbus->swr)
        .item<control::AutomataInputEdge>(bool{}) // mode_change_key
        .item<control::AutomataInputEdge>(bool{}) // vt13_change_button
#ifdef HAS_MINIPC
        .transition<REMOTE_MODE_AUTOPILOT, REMOTE_MODE_FOLLOW>(tranlogic_next_remote_mode_trigger)
#else
        .transition<REMOTE_MODE_ADVANCED, REMOTE_MODE_FOLLOW>(tranlogic_next_remote_mode_trigger)
#endif
        .transition<REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN>(tranlogic_next_remote_mode_trigger)
#ifdef HAS_MINIPC
        .transition<REMOTE_MODE_SPIN, REMOTE_MODE_AUTOPILOT>(tranlogic_next_remote_mode_trigger)
#else
        .transition<REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED>(tranlogic_next_remote_mode_trigger)
#endif
        .build<initial_remote_mode>();

    /*
    todo: should construct more stable logic.
          Example: SHOOT_FRIC_MODE_PREPARING should apply to prevent low-speed shoot.
    */
    // 摩擦轮
    auto tranlogic_fric_wheel_trigger = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& fric_key = COMPONENT(1);
        const auto& vt13_fric_button = COMPONENT(2);
        const auto& control_ready = COMPONENT(3);
        return control_ready.get() && ((swl.downEdge() && swl.get() == remote::UP) || fric_key.upEdge() ||
            vt13_fric_button.upEdge());
    };
    auto tranlogic_control_unavailable = TRANLOGIC {
        return !COMPONENT(3).get();
    };
    auto fric_wheel_aut = control::AutomataBuilder<ShootFricMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputEdge>(bool{}) // friction_key
        .item<control::AutomataInputEdge>(bool{}) // vt13_fric_button
        .item<control::AutomataInputRaw>(bool{}) // control_ready
        .transition<SHOOT_FRIC_MODE_PREPARING, SHOOT_FRIC_MODE_STOP>(tranlogic_control_unavailable)
        .transition<SHOOT_FRIC_MODE_PREPARED, SHOOT_FRIC_MODE_STOP>(tranlogic_control_unavailable)
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
        const auto& vt13_shoot_button = COMPONENT(4);
        const auto& remote_mode = COMPONENT(5);
#ifdef HAS_MINIPC
        const bool minipc_permit = remote_mode.get() != REMOTE_MODE_AUTOPILOT || minipc->target_angle.shoot_cmd;
#else
        const bool minipc_permit = remote_mode.get() != REMOTE_MODE_AUTOPILOT;
#endif
        return (swl.get() == remote::DOWN || shoot_key.get() || vt13_shoot_button.get()) &&
            fric_state.get() == SHOOT_FRIC_MODE_PREPARED &&
            referee_permit.get() &&
            minipc_permit;
    };
    auto shoot_aut = control::AutomataBuilder<ShootMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
        .item<control::AutomataInputRaw>(is_referee_shoot_available)
        .item<control::AutomataInputEdge>(bool{}) // shoot_key
        .item<control::AutomataInputEdge>(bool{}) // vt13_shoot_button
        .item<control::AutomataInputRaw>(initial_remote_mode)
        .transition<SHOOT_MODE_SINGLE, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
        .transition<SHOOT_MODE_BURST, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
        .transition<SHOOT_MODE_STOP, SHOOT_MODE_SINGLE>(tranlogic_shooting_condition)
        .transition<SHOOT_MODE_SINGLE, SHOOT_MODE_BURST>(TRANLOGIC {
            const auto& swl = COMPONENT(0);
            const auto& shoot_key = COMPONENT(3);
            const auto& vt13_shoot_button = COMPONENT(4);
            constexpr uint16_t burst_threshold_cycle = 300;
            return (swl.get() == remote::DOWN && swl.lastUpdate() > burst_threshold_cycle) ||
                (shoot_key.get() && shoot_key.lastUpdate() > burst_threshold_cycle) ||
                (vt13_shoot_button.get() && vt13_shoot_button.lastUpdate() > burst_threshold_cycle);
        })
        .build<SHOOT_MODE_STOP>();

    // clang-format on
    // Automata definitions

    while (true) {
        osDelay(REMOTE_OS_DELAY);

        const bool dbus_online = dbus->IsOnline();
        // Seer 尚未初始化图传串口；未接入 VT13 时按离线处理。
        const bool vt13_c_mode = refereerc != nullptr && refereerc->IsOnline() &&
                                 refereerc->vt13_packet.remote.mode_sw == remote::vt13_remote_t::MODE_C;
        const bool use_vt13 = !dbus_online && vt13_c_mode;
        is_controller_online = dbus_online || vt13_c_mode;

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

        activate_aut.input(std::make_tuple(is_referee_robot_dead, is_controller_online));
        is_activate = activate_aut.state() == ACTIVE;
        const bool control_ready = is_activate && imu->DataReady() && imu->CaliDone();

        // 每帧清空输入，DBUS 优先，离线后才使用 VT13 C 模式。
        remote::switch_t state_r = remote::MID;
        remote::switch_t state_l = remote::MID;
        remote::keyboard_t keyboard{};
        remote::mouse_t mouse{};
        if (control_ready && dbus_online) {
            state_r = dbus->swr;
            state_l = dbus->swl;
            keyboard = dbus->keyboard;
            mouse = dbus->mouse;
        } else if (control_ready && use_vt13) {
            keyboard = refereerc->vt13_packet.keyboard;
            mouse = static_cast<remote::mouse_t>(refereerc->vt13_packet.mouse);
        }

        turbo_edge.input(static_cast<bool>(keyboard.bit.F));
        if (!control_ready) {
            turbo_shoot = false;
        } else if (turbo_edge.posEdge()) {
            turbo_shoot = !turbo_shoot;
        }
        const bool shoot_permitted = control_ready && (is_referee_shoot_available || SHOOT_REFEREE == 0 || turbo_shoot);

        last_remote_mode = remote_mode;
        if (control_ready) {
            remote_mode_aut.input(std::make_tuple(
                state_r,
                static_cast<bool>(keyboard.bit.SHIFT),
                use_vt13 && static_cast<bool>(refereerc->vt13_packet.remote.pause)
            ));
            remote_mode = remote_mode_aut.state();
        } else {
            remote_mode = REMOTE_MODE_STOP;
        }

        fric_wheel_aut.input(std::make_tuple(
            state_l,
            static_cast<bool>(keyboard.bit.Z),
            control_ready && use_vt13 && static_cast<bool>(refereerc->vt13_packet.remote.swl),
            control_ready
        ));
        shoot_fric_wheel_mode = fric_wheel_aut.state();

        shoot_aut.input(std::make_tuple(
            state_l,
            shoot_fric_wheel_mode,
            shoot_permitted,
            static_cast<bool>(mouse.l),
            control_ready && use_vt13 && static_cast<bool>(refereerc->vt13_packet.remote.trigger),
            remote_mode
        ));
        last_shoot_mode = shoot_mode;
        shoot_mode = shoot_aut.state();
    }
}
