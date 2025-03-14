# 拟合数据以使用功率控制模型

**准备**

需要安装以下的包

```
pip install pyserial numpy pandas scipy matplotlib
```

这个示例使用7C制作的电流计进行电流采样，需要使用串口将电流计连接至C板。

**简要步骤**

在开发板中刷入`power_data_collect`目标，连接串口至电脑。该目标使用 <u>DT7 遥控器</u>的 <u>Channel 0</u> 控制一个3508电机，你需要将电流计连接在这一个电调上。

`record.py` 用于从串口识别数据包。使用回车按键以开始/停止记录，记录的数据将储存在csv文件中。

`calc.py` 用于从csv文件中读取记录并拟合参数 k1/k2/k3/k4

现阶段参数写死在`chassis.cpp`中。将拟合到的参数填入，使用`power_limit_test`目标以进行测试。

（文件夹中的脚本均由Copilot生成。）

**开发板->电脑 数据包格式**

```c
struct
{
    uint8_t header[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    float angular_velocity;  // rad/S
    int16_t cmd_current;     // ±16384
    int16_t read_current;    // mA
} data;
```

