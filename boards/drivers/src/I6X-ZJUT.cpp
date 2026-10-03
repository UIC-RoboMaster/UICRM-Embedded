#include "I6X-ZJUT.h"

#include <math.h>
#include <string.h>

I6X i6x;

static uint8_t i6x_rx_dma_buffer[I6X_RX_BUFFER_SIZE] __attribute__((section(".dma_buffer"), aligned(32)));

static int16_t i6x_map_to_660(int16_t val)
{
    if (val >= 0)
    {
        return (int16_t)floorf((660.0f / 783.0f) * (float)val + 0.5f);
    }
    else
    {
        return (int16_t)floorf((660.0f / 784.0f) * (float)val - 0.5f);
    }
}

static int8_t i6x_to_switch(int16_t val)
{
    if (val > 200)
    {
        return I6X_SW_DOWN;
    }
    else if (val < -200)
    {
        return I6X_SW_UP;
    }

    return I6X_SW_MID;
}

static int8_t i6x_to_two_position_switch(int16_t val)
{
    if (val < 0)
    {
        return I6X_SW_2POS_UP;
    }

    return I6X_SW_2POS_DOWN;
}

static int16_t i6x_unpack_channel(const uint8_t *sbus_data, uint8_t index)
{
    uint16_t raw = 1024;

    switch (index)
    {
    case 0:
        raw = (sbus_data[1] | (sbus_data[2] << 8)) & 0x07FF;
        break;
    case 1:
        raw = ((sbus_data[2] >> 3) | (sbus_data[3] << 5)) & 0x07FF;
        break;
    case 2:
        raw = ((sbus_data[3] >> 6) | (sbus_data[4] << 2) | (sbus_data[5] << 10)) & 0x07FF;
        break;
    case 3:
        raw = ((sbus_data[5] >> 1) | (sbus_data[6] << 7)) & 0x07FF;
        break;
    case 4:
        raw = ((sbus_data[6] >> 4) | (sbus_data[7] << 4)) & 0x07FF;
        break;
    case 5:
        raw = ((sbus_data[7] >> 7) | (sbus_data[8] << 1) | (sbus_data[9] << 9)) & 0x07FF;
        break;
    case 6:
        raw = ((sbus_data[9] >> 2) | (sbus_data[10] << 6)) & 0x07FF;
        break;
    case 7:
        raw = ((sbus_data[10] >> 5) | (sbus_data[11] << 3)) & 0x07FF;
        break;
    case 8:
        raw = (sbus_data[12] | (sbus_data[13] << 8)) & 0x07FF;
        break;
    case 9:
        raw = ((sbus_data[13] >> 3) | (sbus_data[14] << 5)) & 0x07FF;
        break;
    default:
        break;
    }

    return (int16_t)((int32_t)raw - 1024);
}

static int16_t i6x_deadband_limit(int16_t val, int16_t deadband)
{
    if (val > -deadband && val < deadband)
    {
        return 0;
    }

    return val;
}

static fp32 i6x_norm_ch(int16_t val)
{
    if (val > I6X_CH_VALUE_MAX)
    {
        val = I6X_CH_VALUE_MAX;
    }
    else if (val < I6X_CH_VALUE_MIN)
    {
        val = I6X_CH_VALUE_MIN;
    }

    return (fp32)val / (fp32)I6X_CH_VALUE_MAX;
}

static void i6x_clear_chassis_command(I6X_Chassis_cmd_t *chassis_cmd)
{
    memset(chassis_cmd, 0, sizeof(I6X_Chassis_cmd_t));
    chassis_cmd->mode = I6X_CHASSIS_ZERO_FORCE;
}

static void i6x_clear_gimbal_command(I6X_Gimbal_cmd_t *gimbal_cmd)
{
    memset(gimbal_cmd, 0, sizeof(I6X_Gimbal_cmd_t));
    gimbal_cmd->mode = I6X_GIMBAL_ZERO_FORCE;
}

static void i6x_clear_command(I6X_Chassis_cmd_t *chassis_cmd, I6X_Gimbal_cmd_t *gimbal_cmd)
{
    i6x_clear_chassis_command(chassis_cmd);
    i6x_clear_gimbal_command(gimbal_cmd);
}

static bool i6x_rc_changed(const I6X_RC_ctrl_t *current, const I6X_RC_ctrl_t *last)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        int16_t diff = current->rc.ch[i] - last->rc.ch[i];
        if (diff < 0)
        {
            diff = -diff;
        }

        if (diff >= I6X_LED_CH_CHANGE_THRESHOLD)
        {
            return true;
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        if (current->rc.s[i] != last->rc.s[i])
        {
            return true;
        }
    }

    return current->rc.frame_lost != last->rc.frame_lost ||
           current->rc.failsafe != last->rc.failsafe ||
           current->online != last->online;
}

static void i6x_start_dma_receive(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t rx_buffer_size)
{
    if (huart == NULL || rx_buffer == NULL || rx_buffer_size == 0U)
    {
        return;
    }

    if (huart->RxState != HAL_UART_STATE_READY)
    {
        HAL_UART_AbortReceive(huart);
    }

    HAL_UART_Receive_DMA(huart, rx_buffer, rx_buffer_size);

    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void I6X::init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size)
{
    (void)Rx_buf;

    memset(&i6x_rc_ctrl, 0, sizeof(i6x_rc_ctrl));
    memset(&last_i6x_rc_ctrl, 0, sizeof(last_i6x_rc_ctrl));
    i6x_clear_command(&chassis_cmd, &gimbal_cmd);

    this->Rx_Buffer = i6x_rx_dma_buffer;
    this->Rx_Buffer_Size = Rx_buf_size;
    i6x_start_dma_receive(huart, this->Rx_Buffer, this->Rx_Buffer_Size);
}

const I6X_RC_ctrl_t *I6X::get_i6x_remote_control_point()
{
    return &i6x_rc_ctrl;
}

I6X_RC_ctrl_t *I6X::get_last_i6x_remote_control_point()
{
    return &last_i6x_rc_ctrl;
}

void I6X::unpack(uint32_t now_ms)
{
    last_i6x_rc_ctrl = i6x_rc_ctrl;

    if (Rx_Buffer[0] != 0x0F || Rx_Buffer[24] != 0x00)
    {
        return;
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        i6x_rc_ctrl.rc.ch[i] = i6x_map_to_660(i6x_unpack_channel(Rx_Buffer, i));
    }

    i6x_rc_ctrl.rc.s[0] = i6x_to_two_position_switch(i6x_unpack_channel(Rx_Buffer, 6));
    i6x_rc_ctrl.rc.s[1] = i6x_to_two_position_switch(i6x_unpack_channel(Rx_Buffer, 7));
    i6x_rc_ctrl.rc.s[2] = i6x_to_switch(i6x_unpack_channel(Rx_Buffer, 8));
    i6x_rc_ctrl.rc.s[3] = i6x_to_two_position_switch(i6x_unpack_channel(Rx_Buffer, 9));

    i6x_rc_ctrl.rc.frame_lost = (Rx_Buffer[23] >> 2) & 0x01;
    i6x_rc_ctrl.rc.failsafe = (Rx_Buffer[23] >> 3) & 0x01;
    i6x_rc_ctrl.last_update_ms = now_ms;
    i6x_rc_ctrl.online = (i6x_rc_ctrl.rc.failsafe == 0) ? 1 : 0;

    if (i6x_rc_changed(&i6x_rc_ctrl, &last_i6x_rc_ctrl))
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    }

    update_command();
}

void I6X::update_online(uint32_t now_ms)
{
    if ((now_ms - i6x_rc_ctrl.last_update_ms) > I6X_REMOTE_TIMEOUT_MS)
    {
        i6x_rc_ctrl.online = 0;
    }

    update_command();
}

void I6X::update_command()
{
    int16_t vx_channel;
    int16_t vy_channel;
    int16_t yaw_channel;
    int16_t pitch_channel;

    if (i6x_rc_ctrl.online == 0 ||
        i6x_rc_ctrl.rc.failsafe != 0)
    {
        i6x_clear_command(&chassis_cmd, &gimbal_cmd);
        return;
    }

    if (i6x_switch_is_up(i6x_rc_ctrl.rc.s[I6X_CHASSIS_MODE_SW]))
    {
        i6x_clear_command(&chassis_cmd, &gimbal_cmd);
        return;
    }
    else if (i6x_switch_is_mid(i6x_rc_ctrl.rc.s[I6X_CHASSIS_MODE_SW]))
    {
        chassis_cmd.mode = I6X_CHASSIS_FREE;
        gimbal_cmd.mode = I6X_GIMBAL_FREE;
    }
    else if (i6x_switch_is_down(i6x_rc_ctrl.rc.s[I6X_CHASSIS_MODE_SW]))
    {
        chassis_cmd.mode = I6X_CHASSIS_TOP;
        gimbal_cmd.mode = I6X_GIMBAL_TOP;
    }
    else
    {
        i6x_clear_command(&chassis_cmd, &gimbal_cmd);
        return;
    }

    chassis_cmd.enable = 1;
    gimbal_cmd.enable = 1;

    vx_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_CHASSIS_VX_CH], I6X_RC_DEADBAND);
    vy_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_CHASSIS_VY_CH], I6X_RC_DEADBAND);
    yaw_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_GIMBAL_YAW_CH], I6X_RC_DEADBAND);
    pitch_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_GIMBAL_PITCH_CH], I6X_RC_DEADBAND);

    chassis_cmd.vx = i6x_norm_ch(vx_channel) * I6X_CHASSIS_MAX_VX;
    chassis_cmd.vy = i6x_norm_ch(vy_channel) * I6X_CHASSIS_MAX_VY;

    gimbal_cmd.yaw_speed = i6x_norm_ch(yaw_channel) *
                            I6X_GIMBAL_MAX_YAW_SPEED;
    gimbal_cmd.pitch_speed = i6x_norm_ch(pitch_channel) *
                              I6X_GIMBAL_MAX_PITCH_SPEED;
}

bool I6X::chassis_switch_is_safe()
{
    return i6x_switch_is_up(i6x_rc_ctrl.rc.s[I6X_CHASSIS_MODE_SW]);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_I6X)
    {
        i6x.unpack(HAL_GetTick());

        i6x_start_dma_receive(I6X_UART, i6x.Rx_Buffer, i6x.Rx_Buffer_Size);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_I6X)
    {
        i6x_start_dma_receive(I6X_UART, i6x.Rx_Buffer, i6x.Rx_Buffer_Size);
    }
}
