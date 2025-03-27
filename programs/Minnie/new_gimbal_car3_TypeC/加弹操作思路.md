```c++
enum ShootFricMode {
   SHOOT_FRIC_MODE_DISABLE = -1,
   SHOOT_FRIC_MODE_STOP = 0,
   SHOOT_FRIC_MODE_PREPARING = 1,
   SHOOT_FRIC_MODE_PREPARED = 2,
};
extern ShootFricMode shoot_flywheel_mode;
enum ShootMode {
   SHOOT_MODE_DISABLE = -1,
   SHOOT_MODE_UNLOAD = -2, // 退弹
   SHOOT_MODE_STOP = 0,    // 停止供弹
   SHOOT_MODE_IDLE = 1,    // 供弹机构待机
   SHOOT_MODE_SINGLE = 2,  // 单发，用于通知shoot_task，发射后由shoot_task设置回IDLE
   SHOOT_MODE_BURST = 3,   // 连发
   SHOOT_MODE_RELOADING = 4, // 补弹
};
```

1. 当ShootFricMode为SHOOT_FRIC_MODE_STOP时,开盖(假设)
2. 当ShootFricMode为SHOOT_FRIC_MODE_PREPARING,按下键盘B开盖
3. 当ShootFricMode为SHOOT_FRIC_MODE_STOP时,DT7的SWL下拨时，开盖
4. 假设能够读取到DT7是否连接到电脑,则当连接到电脑后,通过按下B关闭弹舱盖子(假设)