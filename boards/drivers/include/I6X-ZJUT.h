#ifndef I6X_H
#define I6X_H

#include <stdint.h>
#include "struct_typedef.h"
#include "main.h"
#include "usart.h"

#define I6X_FRAME_LENGTH        25U
#define I6X_RX_BUFFER_SIZE      I6X_FRAME_LENGTH

// iA6B SBUS: 100000 8E2, this project receives it on UART5.
#define UART_I6X                UART5
#define I6X_UART                (&huart5)

#define I6X_CH_VALUE_MAX        660
#define I6X_CH_VALUE_MIN       -660

#define I6X_SW_UP               ((int8_t)1)
#define I6X_SW_MID              ((int8_t)0)
#define I6X_SW_DOWN             ((int8_t)-1)
#define I6X_SW_2POS_UP          ((int8_t)1)
#define I6X_SW_2POS_DOWN        ((int8_t)0)

#define i6x_switch_is_down(s)   ((s) == I6X_SW_DOWN)
#define i6x_switch_is_mid(s)    ((s) == I6X_SW_MID)
#define i6x_switch_is_up(s)     ((s) == I6X_SW_UP)
#define i6x_2pos_switch_is_down(s) ((s) == I6X_SW_2POS_DOWN)
#define i6x_2pos_switch_is_up(s)   ((s) == I6X_SW_2POS_UP)

#define IF_I6X_SW0_UP           i6x_2pos_switch_is_up(i6x.i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW0_DOWN         i6x_2pos_switch_is_down(i6x.i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW1_UP           i6x_2pos_switch_is_up(i6x.i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW1_DOWN         i6x_2pos_switch_is_down(i6x.i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW2_UP           i6x_switch_is_up(i6x.i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_MID          i6x_switch_is_mid(i6x.i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_DOWN         i6x_switch_is_down(i6x.i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW3_UP           i6x_2pos_switch_is_up(i6x.i6x_rc_ctrl.rc.s[3])
#define IF_I6X_SW3_DOWN         i6x_2pos_switch_is_down(i6x.i6x_rc_ctrl.rc.s[3])

#define IF_I6X_SW0_UP_LAST      i6x_2pos_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW0_DOWN_LAST    i6x_2pos_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW1_UP_LAST      i6x_2pos_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW1_DOWN_LAST    i6x_2pos_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW2_UP_LAST      i6x_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_MID_LAST     i6x_switch_is_mid(i6x.last_i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_DOWN_LAST    i6x_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW3_UP_LAST      i6x_2pos_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[3])
#define IF_I6X_SW3_DOWN_LAST    i6x_2pos_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[3])

#define I6X_SW_A                0
#define I6X_SW_B                1
#define I6X_SW_C                2
#define I6X_SW_D                3

#define I6X_GIMBAL_YAW_CH       0
#define I6X_GIMBAL_PITCH_CH     1
#define I6X_CHASSIS_VY_CH       2
#define I6X_CHASSIS_VX_CH       3

#define I6X_CHASSIS_MODE_SW     I6X_SW_C

#define I6X_RC_DEADBAND         20
#define I6X_LED_CH_CHANGE_THRESHOLD 20
#define I6X_REMOTE_TIMEOUT_MS   100U

#define I6X_CHASSIS_MAX_VX      3.0f
#define I6X_CHASSIS_MAX_VY      3.0f
#define I6X_GIMBAL_MAX_YAW_SPEED   3.5f
#define I6X_GIMBAL_MAX_PITCH_SPEED 1.5f

typedef enum
{
        I6X_CHASSIS_ZERO_FORCE = 0,
        I6X_CHASSIS_FREE,
        I6X_CHASSIS_TOP,
} i6x_chassis_mode_e;

typedef enum
{
        I6X_GIMBAL_ZERO_FORCE = 0,
        I6X_GIMBAL_TOP,
        I6X_GIMBAL_FREE,
} i6x_gimbal_mode_e;

typedef struct
{
        struct
        {
                int16_t ch[6];
                int8_t s[4];
                uint8_t frame_lost;
                uint8_t failsafe;
        } rc;

        uint32_t last_update_ms;
        uint8_t online;

} I6X_RC_ctrl_t;

typedef struct
{
        fp32 vx;
        fp32 vy;
        uint8_t enable;
        uint8_t mode;
} I6X_Chassis_cmd_t;

typedef struct
{
        fp32 yaw_speed;
        fp32 pitch_speed;
        uint8_t enable;
        uint8_t mode;
} I6X_Gimbal_cmd_t;

class I6X
{
public:
    I6X_RC_ctrl_t i6x_rc_ctrl;
    I6X_RC_ctrl_t last_i6x_rc_ctrl;
    I6X_Chassis_cmd_t chassis_cmd;
    I6X_Gimbal_cmd_t gimbal_cmd;

    uint8_t *Rx_Buffer;
    uint16_t Rx_Buffer_Size;

    void init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size);

    const I6X_RC_ctrl_t *get_i6x_remote_control_point();
    I6X_RC_ctrl_t *get_last_i6x_remote_control_point();

    void unpack(uint32_t now_ms);
    void update_online(uint32_t now_ms);
    void update_command();
    bool chassis_switch_is_safe();
};

extern I6X i6x;

#endif
