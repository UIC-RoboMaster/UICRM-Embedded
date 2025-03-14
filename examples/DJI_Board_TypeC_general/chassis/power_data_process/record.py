import serial
import time
import csv
import threading
import datetime
from serial.serialutil import SerialException
from frame_parser import *

port = "COM8"
baud_rate = 921600

# 记录控制
recording = False
recorded_data = []
record_lock = threading.Lock()

def keyboard_monitor():
    """监视键盘输入，用于控制数据记录"""
    global recording, recorded_data
    
    print("按Enter键开始记录数据，再次按Enter键停止记录并保存数据")
    
    while True:
        input()  # 等待Enter键被按下
        
        with record_lock:
            if not recording:
                recording = True
                recorded_data = []  # 清空之前的数据
                print("开始记录数据...")
            else:
                recording = False
                save_to_csv(recorded_data)
                print("停止记录数据，数据已保存")

def save_to_csv(data_list):
    """将记录的数据保存到CSV文件"""
    if not data_list:
        print("没有数据需要保存")
        return
    
    # 创建包含日期和时间的文件名
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"data_record_{timestamp}.csv"
    
    # 获取数据字段名称
    fieldnames = list(data_list[0].keys())
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data_list)
        print(f"数据已保存到 {filename}")
    except Exception as e:
        print(f"保存数据时出错: {e}")

def read_serial_data():
    """从串口读取数据并解析数据帧"""
    global recording, recorded_data
    
    try:
        # 启动键盘监控线程
        keyboard_thread = threading.Thread(target=keyboard_monitor, daemon=True)
        keyboard_thread.start()
        
        # 打开串口
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"成功打开串口 {port}")
        
        # 数据缓冲区
        buffer = bytes()
        
        while True:
            if ser.in_waiting > 0:
                # 读取可用的数据
                data = ser.read(ser.in_waiting)
                buffer += data
                
                # 查找并解析帧
                parsed_frames, buffer = find_and_parse_frames(buffer)
                
                # 处理解析出的数据
                for frame in parsed_frames:
                    print(f"解析到数据: 角速度={frame['angular_velocity']:>8.2f} "
                          f"命令电流={frame['raw_current']:>8} "
                          f"读取电流={frame['read_current']:>8}")
                    
                    # 如果正在记录，保存到记录列表
                    with record_lock:
                        if recording:
                            # 添加时间戳到数据
                            frame['timestamp'] = time.time()
                            recorded_data.append(frame)
            
            # 短暂休眠，减少CPU使用率
            time.sleep(0.01)
            
    except SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("程序退出")
    finally:
        # 如果程序退出时正在记录，保存已记录的数据
        with record_lock:
            if recording and recorded_data:
                save_to_csv(recorded_data)
                print("程序退出，已保存当前记录的数据")
        
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == "__main__":
    read_serial_data()
