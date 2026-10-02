#ifndef I6X_H
#define I6X_H

#include <stdint.h>

#define I6X_FRAME_LENGTH 25u

/* RC Switch Definition */
#define I6X_SW_UP    ((int8_t)1)
#define I6X_SW_MID   ((int8_t)0)
#define I6X_SW_DOWN  ((int8_t)-1)
#define i6x_switch_is_down(s) (s == I6X_SW_DOWN)
#define i6x_switch_is_mid(s)  (s == I6X_SW_MID)
#define i6x_switch_is_up(s)   (s == I6X_SW_UP)

/* Data Struct */
typedef struct {
    int16_t ch[6];      // 6个通道数据
    int8_t  s[4];       // 四个拨杆数据
    uint8_t failsafe;   // 失控标志位
    uint8_t frame_lost; // 丢帧标志位
} __attribute__((packed)) i6x_ctrl_t;

void sbus_to_i6x(i6x_ctrl_t* i6x_ctrl, const uint8_t *sbus_data);
i6x_ctrl_t* get_i6x_point(void);

#endif // I6X_H