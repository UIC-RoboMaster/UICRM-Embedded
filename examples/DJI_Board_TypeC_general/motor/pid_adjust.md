# PID调参
### Serial Plot
https://hackaday.io/project/5334-serialplot-realtime-plotting-software

**SerialPlot v0.12 release**  
Hasan Yavuz Özderya • 05/14/2021 at 16:19 • 3 comments  
As always you can find latest download links in the sidebar.

Download links for v0.12:  
Windows installer: https://serialplot.ozderya.net/downloads/serialplot-0.12.0-win32-setup.exe  
Windows EXE 7Zip: https://serialplot.ozderya.net/downloads/serialplot-0.12.0-win32.7z  
Windows EXE: https://serialplot.ozderya.net/downloads/serialplot-0.12.0-win32.exe  
Linux AppImage (based on Ubuntu 18.04): https://serialplot.ozderya.net/downloads/serialplot-0.12.0-x86_64.AppImage

### 如何使用
1. 下载安装SerialPlot
2. 加载配置文件`pid_adjust.ini`
3. 先启用并调整OMEGA PID(速度->电流)、再加上THETA PID(位置->速度)
4. 注意代码中的硬件外设huart、波特率(921600)、串口发送数据的频率(500Hz)
4. 默认的包参数为：包头(AA BB)、数据类型(float)、数据个数(5)

### 如何调参
TODO