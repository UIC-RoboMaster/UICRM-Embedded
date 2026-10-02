**关于新遥控器的相关思路**
Author: BillZH/JiaHeWG

目前已经能够确定的主要内容：
1. i6x系列接收机使用SBUS解码，与DBUS电气特性相同，如果使用现有DR16接收机的主控可以直接改接，需要在STM32CubeMX中修改对应Layout **（待验证）**

![cubemx-config-boardC-USART-remote.png](cubemx-config-boardC-USART-remote.png)
2. 已经阅读了对应代码和初步回顾C++，需要做的主要工作即为调整dji_dbus.cpp/.h及dji_remote.h三个文件，以实现暴露接口相同，最小化后续驱动影响。可能可以考虑利用现有闲置sbus.cpp完成？
3. 这套新遥控器硬件限制无法完成键盘鼠标模拟，暂无可选替代，待进一步确定。

接下来需要等待遥控器及接收机到货后找一块闲置C板/现有主控测试输出，适配即可。

Version: 2026.10.03