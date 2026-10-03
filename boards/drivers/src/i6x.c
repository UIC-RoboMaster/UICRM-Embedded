/*********************************
 * @file: i6x.c/.h
 * @author: X.yu
 * @date: 2025年7月23日
 * @brief: FS-i6x遥控器搭配iA6B接收机sbus数据解包，可直接平替dt7部分
 * @note: 该文件实现了对i6x遥控器的数据解包。
 * 四个摇杆通道值初始值为-784~783，为方便适配现有代码进行了对-660~660的映射，默认开启
 * 可直接改变宏定义MAPPING_ENABLE实现映射开关
 * i6x包括四个遥控通道，两个旋钮通道，*三个两档拨杆，一个三档拨杆*
 * @copyright: Copyright (c) 2025
 * @license: MIT
 *********************************/
#include "i6x.h"
#include <math.h>

/*
 * 拨杆 s[0] ~ s[3]
 * 三段式归一化为 1 / 0 / -1
 * 想要对调 -1 和 1，只需要把括号里的 < 和 > 对调一下
 */
#define TO_STICK(v)  (((v) < 0) - ((v) > 0))

/* 摇杆通道值映射宏定义开关 */
#define MAPPING_ENABLE 1

i6x_ctrl_t i6x_ctrl;

/**
 * @brief 为方便适配代码增加该映射-660~660函数，原始数据为-784~783
 * @param val i6x搭配iA6B接收器解包后的初始值
 * @return 映射到-660~660的值
 */
static int16_t map_to_660(const int16_t val) {
    if (val >= 0)
        return (int16_t) floorf((660.0f / 783.0f) * (float) val + 0.5f);
    else
        return (int16_t) floorf((660.0f / 784.0f) * (float) val + 0.5f);
}

void sbus_to_i6x(i6x_ctrl_t *i6x_ctrl, const uint8_t *sbus_data) {
    // 起始字节、结束字节检查
    if (sbus_data[0] != 0x0F || sbus_data[24] != 0x00) {
        return;
    }

    // 解包
    i6x_ctrl->ch[0] = (int16_t)(((sbus_data[1] | (sbus_data[2] << 8)) & 0x07FF) - 1024);
    i6x_ctrl->ch[1] = (int16_t)((((sbus_data[2] >> 3) | (sbus_data[3] << 5)) & 0x07FF) - 1024);
    i6x_ctrl->ch[2] = (int16_t)((((sbus_data[3] >> 6) | (sbus_data[4] << 2) | (sbus_data[5] << 10)) & 0x07FF) - 1024);
    i6x_ctrl->ch[3] = (int16_t)((((sbus_data[5] >> 1) | (sbus_data[6] << 7)) & 0x07FF) - 1024);
    i6x_ctrl->ch[4] = (int16_t)((((sbus_data[6] >> 4) | (sbus_data[7] << 4)) & 0x07FF) - 1024);
    i6x_ctrl->ch[5] = (int16_t)((((sbus_data[7] >> 7) | (sbus_data[8] << 1) | (sbus_data[9] << 9)) & 0x07FF) - 1024);

    i6x_ctrl->s[0] = (int8_t) TO_STICK((((sbus_data[9] >> 2) | (sbus_data[10] << 6)) & 0x07FF) - 1024);
    i6x_ctrl->s[1] = (int8_t) TO_STICK((((sbus_data[10] >> 5) | (sbus_data[11] << 3)) & 0x07FF) - 1024);
    i6x_ctrl->s[2] = (int8_t) TO_STICK(((sbus_data[12] | (sbus_data[13] << 8)) & 0x07FF) - 1024);
    i6x_ctrl->s[3] = (int8_t) TO_STICK((((sbus_data[13] >> 3) | (sbus_data[14] << 5)) & 0x07FF) - 1024);

    // 通道值映射
#if MAPPING_ENABLE
    for (int i = 0; i < 6; i++) {
        i6x_ctrl->ch[i] = map_to_660(i6x_ctrl->ch[i]);
    }
#endif

    // 失控丢帧标志位，遥控器断连后先后置1
    const uint8_t flag = sbus_data[23];
    i6x_ctrl->frame_lost = (flag >> 2) & 0x01;
    i6x_ctrl->failsafe   = (flag >> 3) & 0x01;
}

/**
 * @brief 获取存放数据结构体指针
 * @return i6x遥控器数据结构体指针
 */
i6x_ctrl_t *get_i6x_point(void) {
    return &i6x_ctrl;
}