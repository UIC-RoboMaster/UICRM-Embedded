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

#include "remote_task.h"

#include <string.h>
#include "Automata.h"
#include "AutomataSystem.h"
#include "imu_task.h"

remote::DBUS* dbus = nullptr;
RemoteMode remote_mode = REMOTE_MODE_FOLLOW;
ShootFricMode shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
ShootMode shoot_load_mode = SHOOT_MODE_STOP;
bool turbo_shoot = false;

void init_dbus() {
    dbus = new remote::DBUS(&huart3);
}

osThreadId_t remoteTaskHandle;

void remoteTask(void* arg) {
    // 接口转换
    UNUSED(arg);
    osDelay(1000);

    // 遥控器是否在线
    bool is_controller_online = false;
    // 裁判系统端机器人是否阵亡
    bool is_referee_robot_dead = true;
    // 裁判系统端是否允许射击
    bool is_referee_shoot_available = false;

    remote::switch_t state_r = remote::MID;
    remote::switch_t state_l = remote::MID;
    remote::keyboard_t keyboard{};
    remote::mouse_t mouse{};
    BoolEdgeDetector turbo_edge{false};

    // clang-format off

    // 使/失能（上/下电）
    auto tranlogic_active_condition = TRANLOGIC {
        const auto& referee_cmd_dead = COMPONENT(0);
        const auto& control_online = COMPONENT(1);
        // 遥控器在线 && 右摇杆不在最下档 && 裁判系统反馈 未阵亡
        return control_online.get()
        && (!dbus->IsOnline() || dbus->swr != remote::DOWN)
        && !referee_cmd_dead.get();
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
        return (swr.downEdge() && swr.get() == remote::UP)
        || mode_change_key.upEdge()
        || vt13_change_button.upEdge();
    };
    auto remote_mode_aut = control::AutomataBuilder<RemoteMode>()
        .item<control::AutomataInputEdge>(dbus->swr)
        .item<control::AutomataInputEdge>(bool{}) // mode_change_key
        .item<control::AutomataInputEdge>(bool{}) // vt13_change_button
        .transition<REMOTE_MODE_FOLLOW, REMOTE_MODE_SPIN>(tranlogic_next_remote_mode_trigger)
        .transition<REMOTE_MODE_SPIN, REMOTE_MODE_ADVANCED>(tranlogic_next_remote_mode_trigger)
        .transition<REMOTE_MODE_ADVANCED, REMOTE_MODE_FOLLOW>(tranlogic_next_remote_mode_trigger)
        .build<REMOTE_MODE_FOLLOW>();

    // 摩擦轮
    auto tranlogic_fric_wheel_trigger = TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& fric_key = COMPONENT(1);
        const auto& vt13_fric_button = COMPONENT(2);
        return (swl.downEdge() && swl.get() == remote::UP)
        || fric_key.upEdge()
        || vt13_fric_button.upEdge();
    };
    auto fric_wheel_aut = control::AutomataBuilder<ShootFricMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputEdge>(bool{}) // friction_key
        .item<control::AutomataInputEdge>(bool{}) // vt13_fric_button
        // 加速中 → 停
        .transition<SHOOT_FRIC_MODE_PREPARING, SHOOT_FRIC_MODE_STOP>(tranlogic_fric_wheel_trigger)
        // 就绪 → 停
        .transition<SHOOT_FRIC_MODE_PREPARED, SHOOT_FRIC_MODE_STOP>(tranlogic_fric_wheel_trigger)
        // 停 → 加速中
        .transition<SHOOT_FRIC_MODE_STOP, SHOOT_FRIC_MODE_PREPARING>(tranlogic_fric_wheel_trigger)
        // 加速中 → 就绪 无条件 永远满足 md什么脑残逻辑
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
        return (swl.get() == remote::DOWN || shoot_key.get() || vt13_shoot_button.get()) &&
            fric_state.get() == SHOOT_FRIC_MODE_PREPARED &&
            referee_permit.get();
    };
    auto shoot_aut = control::AutomataBuilder<ShootMode>()
        .item<control::AutomataInputEdge>(dbus->swl)
        .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
        .item<control::AutomataInputRaw>(is_referee_shoot_available)
        .item<control::AutomataInputEdge>(bool{}) // shoot_key
        .item<control::AutomataInputEdge>(bool{}) // vt13_shoot_button
        .transition<SHOOT_MODE_SINGLE, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
        .transition<SHOOT_MODE_STOP, SHOOT_MODE_SINGLE>(tranlogic_shooting_condition)
        .build<SHOOT_MODE_STOP>();

    // clang-format on

    while (true) {
        const bool is_dbus_offline = (!dbus->IsOnline()) || (dbus->swr == remote::DOWN);
        // VT13必须在线且处于C模式才算可控
        const bool is_vt13_offline =
            !(refereerc->IsOnline() && (refereerc->vt13_packet.remote.mode_sw == remote::vt13_remote_t::MODE_C));

        is_controller_online = !is_dbus_offline || !is_vt13_offline;
#ifdef HAS_REFEREE
        is_referee_robot_dead = referee->game_robot_status.remain_HP == 0;
        is_referee_shoot_available = (referee->game_robot_status.shooter_heat_limit -
                                      referee->power_heat_data.shooter_id1_42mm_cooling_heat) >= 100 &&
                                     imu->CaliDone();
#else
        is_referee_robot_dead = false;
        is_referee_shoot_available = true;
#endif
        // DBUS和VT13都离线，或者机器人死亡，才进入安全模式

        activate_aut.input(std::make_tuple(is_referee_robot_dead, is_controller_online));
        const bool is_activate = activate_aut.state() == ACTIVE;

        if (!is_activate) {
            // 替代原来的 is_killed 分支
            remote_mode = REMOTE_MODE_KILL;
            shoot_load_mode = SHOOT_MODE_DISABLE;
            shoot_flywheel_mode = SHOOT_FRIC_MODE_STOP;
            osDelay(REMOTE_OS_DELAY);
            continue;
        }

        // 每帧先清零，再按当前在线的控制源更新，避免残留上一帧键鼠输入
        state_r = remote::MID;
        state_l = remote::MID;
        memset(&keyboard, 0, sizeof(keyboard));
        memset(&mouse, 0, sizeof(mouse));

        const bool vt13_c_mode =
            refereerc->IsOnline() && (refereerc->vt13_packet.remote.mode_sw == remote::vt13_remote_t::MODE_C);

        if (dbus->IsOnline()) {  // DBUS
            state_r = dbus->swr;
            state_l = dbus->swl;
            keyboard = dbus->keyboard;
            mouse = dbus->mouse;
        } else if (vt13_c_mode) {
            keyboard = refereerc->vt13_packet.keyboard;
            mouse = (remote::mouse_t)refereerc->vt13_packet.mouse;
        }

        turbo_edge.input(keyboard.bit.F);
        if (turbo_edge.posEdge()) {
            turbo_shoot = !turbo_shoot;
        }

        const bool shoot_permitted = is_referee_shoot_available || SHOOT_REFEREE == 0 || turbo_shoot;

        remote_mode_aut.input(
            std::make_tuple(
                state_r,
                static_cast<bool>(keyboard.bit.SHIFT),
                vt13_c_mode && static_cast<bool>(refereerc->vt13_packet.remote.pause)
            )
        );
        remote_mode = remote_mode_aut.state();

        fric_wheel_aut.input(
            std::make_tuple(
                state_l,
                static_cast<bool>(keyboard.bit.Z),
                vt13_c_mode && static_cast<bool>(refereerc->vt13_packet.remote.swl)
            )
        );
        shoot_flywheel_mode = fric_wheel_aut.state();

        shoot_aut.input(
            std::make_tuple(
                state_l,
                shoot_flywheel_mode,
                shoot_permitted,
                static_cast<bool>(mouse.l),
                vt13_c_mode && static_cast<bool>(refereerc->vt13_packet.remote.trigger)
            )
        );
        shoot_load_mode = shoot_aut.state();

        osDelay(REMOTE_OS_DELAY);
    }
}

void init_remote() {
    init_dbus();
}