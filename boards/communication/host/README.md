# Host 通信协议（MCU ↔ 上位机）

MCU 与 minipc 之间的一条全双工链路。线上同时跑两路数据：

1. **host 自有帧**：MCU 状态上行（0x05xx）、控制指令下行（0x04xx）
2. **裁判原始帧**：MCU 把裁判系统串口收到的整帧原样转发，minipc 按官方协议自行解析

两路共用同一条串口/USB，接收方靠 `cmd_id` 号段区分。

| 项 | 值 |
| --- | --- |
| 协议版本 | `HOST_PROTOCOL_VERSION = 2`（随 0x0500 下发） |
| 线格式 | 与裁判系统一致：`A5` 帧头 + CRC8 + CRC16 |
| 物理层 | UART 921600 8N1 / USB CDC 虚拟串口 |
| 实现 | `boards/communication/host`（MCU 侧）、`boards/communication/referee`（裁判侧） |

---

## 1. 帧格式

```
偏移  0      1      2      3      4      5      6     7..6+n   7+n  8+n
     A5   len_L  len_H  seq  CRC8  cmd_L  cmd_H   data     CRC16_L CRC16_H
     └──────── 帧头 5 字节 ───────┘ └ cmd 2 ┘ └ n ┘  └─── 2 ───┘
```

| 字段 | 说明 |
| --- | --- |
| `A5` | 起始字节，固定 |
| `len` | **data 段**字节数，小端，不含头尾 |
| `seq` | 发送方包序号，逐帧 +1，回绕 |
| `CRC8` | 覆盖帧头 5 字节（SOF..seq），初值 `0xFF`，多项式 `x^8+x^5+x^4+1` |
| `cmd_id` | 命令码，小端 |
| `CRC16` | 覆盖 `SOF` 到 data 末尾（即整帧减最后 2 字节），初值 `0xFFFF`，多项式 `x^16+x^12+x^5+1` |

单帧上限 320 字节（含头尾），data 段上限 311 字节。

### 接收建议

- 逐字节找 `A5`，凑够 5 字节先验 CRC8，通过后再按 `len` 等齐整帧验 CRC16；
- 任何一步失败都从下一字节重新找 `A5`，不要丢弃整个缓冲区；
- `len > 311` 直接判非法帧。

---

## 2. 命令码

| cmd_id | 方向 | 名称 | data 长度 | 周期 | 可丢 |
| --- | --- | --- | --- | --- | --- |
| `0x0400` | minipc → MCU | 自瞄控制 | 9 | 500 Hz | 否 |
| `0x0401` | minipc → MCU | 导航控制 | 16 | 50~100 Hz | 否 |
| `0x0500` | MCU → minipc | 版本与构型 | 10 | 1 Hz | 可 |
| `0x0501` | MCU → minipc | 机器人状态 | 6 | 1 Hz + 变化即发 | 否 |
| `0x0502` | MCU → minipc | 底盘状态 | 17 | 100 Hz | 否 |
| `0x0503` | MCU → minipc | 云台姿态与自瞄使能 | 22 | 500 Hz | 否 |
| `0x0504` | MCU → minipc | 发射状态 | 5 | 50 Hz | 可 |
| `0x0505` | MCU → minipc | 调试日志 | 62 | 事件驱动 | 可（**尚未实现**） |
| `0x0001`~`0x0A06` | MCU → minipc | 裁判原始帧转发 | 原帧 | 随裁判系统 | 事件帧不可丢 |

规则：

- 长度必须**严格等于**表中值，多一字节少一字节都算版本失配，整帧丢弃；
- 未知 `cmd_id` 静默跳过，不得据此中断解析；
- 已删除的号位不重用：`0x0403`（原 NO_TARGET_FLAG）、`0x0404`（原 SHOOT_CMD）已并入 `0x0400`；
- `0x0401` 原为 PACK 调试载荷，v2 起改为导航控制，旧载荷不再兼容。

---

## 3. 下行帧（minipc → MCU）

### 0x0400 自瞄控制 · 9 字节

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | `float` | `pitch` | rad | 世界系绝对 pitch |
| 4 | `float` | `yaw` | rad | 世界系绝对 yaw |
| 8 | `uint8` | `shoot` | — | 1 允许发弹，0 禁止 |

- 角度是**绝对值**，不是增量，MCU 直接作为闭环目标，不需要累加；
- 参考系与 `0x0503` 的 `pitch_angle/yaw_angle` 相同（IMU 世界系）；
- `shoot` 只是意图。MCU 还要结合云台/发射使能、摩擦轮达速、弹量、热量自行裁决；
- 超过 **100 ms** 未收到，MCU 必须停火并标记目标失效。

### 0x0401 导航控制 · 16 字节

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | `float` | `vx` | m/s | 体坐标系前进速度 |
| 4 | `float` | `vy` | m/s | 体坐标系左移速度，**发送端已取反** |
| 8 | `float` | `wz` | rad/s | 转向角速度，逆时针为正 |
| 12 | `float` | `target_angle` | rad | 目标航向角，当前固定 0（保留） |

- `vx/vy/wz` 与 `0x0502` 反馈同向（x 前、y 左、逆时针为正）；
- `vy` 的符号约定沿用旧 `SerialBoard::send`：发送端下发前取反，接收端按此约定解释，**不要再取反一次**；
- 闭环所需的底盘航向反馈见 `0x0502` 的 `feedback_angle`；
- 超过 **200 ms** 未收到，MCU 应按斜坡把速度归零，而不是硬切 0。

---

## 4. 上行帧（MCU → minipc）

### 0x0500 版本与构型 · 10 字节 · 1 Hz

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | `uint8` | `host_version` | 协议版本，当前 2 |
| 1 | `uint16` | `referee_season` | 2026 / 2027 |
| 3 | `uint8` | `robot_type` | 1 英雄，3/4 步兵，7 哨兵，9 雷达 |
| 4 | `uint8` | `chassis_type` | 0 无，1 mecanum，2 omni，3 steering，4 swerve，5 legs |
| 5 | `uint8` | `gimbal_type` | 0 无，1 gimbal，2 大小 yaw，3 折叠 yaw |
| 6 | `uint32` | `uptime_ms` | 本板运行时间，ms |

- 收不到首帧视为链路未建立；`uptime_ms` 回落 = MCU 重启；
- `chassis_type` / `gimbal_type` 决定后续帧里哪些字段有效。

### 0x0501 机器人状态 · 6 字节 · 1 Hz + 变化即发

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | `uint8` | `enable` | 位域：bit0 底盘，bit1 云台，bit2 发射，bit3 超电，bit4~7 保留 |
| 1 | `uint8` | `controller_online` | 遥控是否激活 |
| 2 | `uint8` | `robot_mode` | 0 无，1 跟随，2 小陀螺，3 全自动哨兵，4 坦克，5 高级调试 |
| 3 | `uint8` | `fric_mode` | 0 停，1 加速中，2 就绪 |
| 4 | `uint8` | `shoot_mode` | 0 stop，1 single，2 burst |
| 5 | `uint8` | `module_online` | 在线位域：bit0 底盘，bit1 云台，bit2 发射，bit3 IMU，bit4 裁判，bit5 超电 |

本帧的 `enable` 是 MCU 状态机的软件使能；裁判系统各电源口使能在转发帧 `0x0201` 里，两者都满足才视为模块可用。血量、击杀、弹量、热量同样走裁判转发，不在这里重复。

### 0x0502 底盘状态 · 17 字节 · 100 Hz

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | `uint8` | `chassis_enabled` | 0/1 |
| 1 | `float` | `speed_x` | 实测前进速度，m/s |
| 5 | `float` | `speed_y` | 实测左移速度，m/s |
| 9 | `float` | `speed_turn` | 实测转向角速度，rad/s |
| 13 | `float` | `feedback_angle` | 底盘航向角反馈，rad，世界系（IMU 零位） |

`feedback_angle` 不是 IMU 的 yaw：小陀螺模式下两者相差云台相对角，导航闭环必须用本字段。

### 0x0503 云台姿态与自瞄使能 · 22 字节 · 500 Hz

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | `uint8` | `gimbal_enable` | 0 关闭，1 使能 |
| 1 | `uint8` | `autoaim_enabled` | 1 自瞄激活 |
| 2 | `float` | `pitch_angle` | rad，IMU 世界系 |
| 6 | `float` | `yaw_angle` | rad，IMU 世界系 |
| 10 | `float` | `roll_angle` | rad，IMU 世界系 |
| 14 | `float` | `upper_yaw_angle` | 仅 `gimbal_type=2` 有效，其余置 0 |
| 18 | `float` | `fold_yaw_angle` | 仅 `gimbal_type=3` 有效，其余置 0 |

- 这是唯一延迟敏感的上行流，发送调度里应最先发出；
- `autoaim_enabled` 是 MCU 的最终裁决，minipc 只服从不自裁；
- `gimbal_enable=0` 时电机不驱动、角度仍会更新，不应据此激活自瞄。

### 0x0504 发射状态 · 5 字节 · 50 Hz

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | `uint8` | `shoot_enable` | 0/1 |
| 1 | `uint8` | `fric_state` | 实测：0 停，1 加速中，2 已达速 |
| 2 | `uint8` | `load_state` | 拨弹轮：0 停，1 正常，2 卡弹 |
| 3 | `uint16` | `shot_count` | 累计发弹数，上电清零 |

开火判断用 `fric_state`（实测），不要用 `0x0501` 的 `fric_mode`（控制意图）。

### 0x0505 调试日志 · 62 字节 · 事件驱动

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | `uint8` | `level` | 0 debug，1 info，2 warn，3 error |
| 1 | `uint8` | `length` | text 有效字节数，0~60 |
| 2 | `char[60]` | `text` | UTF-8，非 NUL 结尾 |

字段已定义，MCU 侧暂未实现发送；接收方应能解析但不应依赖。

### 裁判原始帧转发

MCU 把裁判串口上通过 CRC16 的整帧原样写入本链路，`cmd_id`、`seq`、CRC 全部保持裁判系统的原值，不重新编码。

- 范围 `0x0001`~`0x0A06`，与 host 的 `0x04xx/0x05xx` 不重叠；
- 事件类（`0x0101` 事件、`0x0206` 受击、`0x0104` 警告）不可丢；
- 周期状态类（`0x0201/0x0202/0x0207/0x0208` 等）丢失一帧无影响，接收方按最新值使用。

---

## 5. 时序与失效

| 项 | 阈值 | 行为 |
| --- | --- | --- |
| `0x0400` 失效 | 100 ms | MCU 停止发弹，自瞄目标标记失效 |
| `0x0401` 失效 | 200 ms | 底盘速度按斜坡归零，回落遥控 |
| minipc 在线判定 | 500 ms | MCU 侧 `IsOnline()` 为 false |
| MCU 失联判定 | 2 s 未收到 `0x0500` | minipc 侧自行判定 |

约定：

- 下行指令是**绝对值**，minipc 必须周期下发；MCU 不记忆历史指令，超时即失效；
- 目标丢失时 minipc 应停止发送 `0x0400` 或把 `shoot` 置 0，但 MCU 侧仍必须实现超时保护，不能依赖对端主动停止；
- 链路恢复后 MCU 从收到下一个合法帧开始重新计时。

---

## 6. MCU 侧接入

```cpp
// 初始化：先建链路，再建 Host，最后把 Host 作为裁判转发的下游
auto* uart = new bsp::UART(&huart6);
uart->SetBaudrate(921600);
uart->SetupRx(1024);
uart->SetupTx(1024);

auto* link = new host_comm::UartLink(uart);
auto* host = new host_comm::Host(*link);
auto* referee = new referee_comm::Referee(referee_uart, protocol, host);

// 1 kHz 任务里：填数据 + 推一拍
void minipc_task(void*) {
    while (true) {
        host->gimbal_status.pitch_angle = ...;
        host->Pump(bsp::GetHighresTickMilliSec());
        osDelay(1);
    }
}

// 控制任务里：读下行，先判时效
if (host->AimValid()) {
    gimbal.SetTarget(host->aim.pitch, host->aim.yaw);
    shoot_allowed = host->aim.shoot;
}
```

约束：

- `Pump()` 是整条链路的**唯一写者**，只能由一个任务调用；
- 裁判帧由 `Referee` 在自己的接收线程里投递，`Pump()` 负责按优先级发出；
- 上行结构体由应用任务填写、`Pump()` 读取，没有加锁，允许偶发「半新半旧」，控制环本身对此不敏感。

---

## 7. 上位机解析示例（Python 伪代码）

```python
def feed(buf):
    while True:
        i = buf.find(0xA5)
        if i < 0:
            return
        del buf[:i]
        if len(buf) < 5:
            return
        ln = buf[1] | (buf[2] << 8)
        if crc8(buf[:4]) != buf[4] or ln > 311:
            del buf[:1]          # 假帧头，继续找
            continue
        total = 5 + 2 + ln + 2
        if len(buf) < total:
            return
        if crc16(buf[:total - 2]) != (buf[-2] | (buf[-1] << 8)):
            del buf[:1]
            continue
        cmd = buf[5] | (buf[6] << 8)
        on_frame(cmd, buf[7:7 + ln])
        del buf[:total]
```

解析要点：

- 用**滑动窗口**处理半包，不要假设一次 `read()` 正好是一帧；
- `len` 一旦超过 311 直接当作噪声，否则会无限等待；
- 裁判转发帧与 host 帧在同一个流里，按 `cmd_id` 分流即可。

---

## 8. 兼容规则

1. **append-only**：已有字段的位置、类型、含义不改不删，新字段追加在末尾；
2. 帧长变化 = 版本失配，必须同时提升 `HOST_PROTOCOL_VERSION`；
3. `cmd_id` 删除后不重用，避免新旧固件混跑时语义错位；
4. 接收方遇到未知 `cmd_id` 或长度不符时丢弃该帧，继续解析后续字节；
5. 版本差异以 `0x0500` 的 `host_version` 为准，不要靠帧长猜测。

---

## 9. 联调排查

| 现象 | 优先检查 |
| --- | --- |
| 完全收不到字节 | 线序/波特率；USB 是否枚举成串口（设备管理器）；`0x0500` 是否在发 |
| 收到字节但全是乱码 | 波特率不匹配；CRC 参数用错（初值 0xFF / 0xFFFF） |
| 只能收到 `0x05xx` | 裁判串口没接或没数据，转发链路正常 |
| 只能收到裁判帧 | MCU 侧 `Pump()` 没被调用，或调度表被改坏 |
| 上行有、下行无 | PC 侧 `cmd_id` 写错（v2 是 `0x0400/0x0401`）；或帧长不是 9/16 |
| 偶发丢帧 | 检查 MCU 侧 `stats().tx_retry`；UART 发送环过小或上位机读得太慢 |
| 数据忽新忽旧 | 正常现象：结构体无锁共享；关键量请用单帧内自洽的判断 |
