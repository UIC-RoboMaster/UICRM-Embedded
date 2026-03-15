import argparse
import socket
import subprocess
import time
import sys
import os
import signal
import threading


OPENOCD_PORT = 4444
RTT_PORT = 8888


def is_port_open(port, host="127.0.0.1"):
    try:
        with socket.create_connection((host, port), timeout=1):
            return True
    except Exception:
        return False

def send_commands(port, commands, host="127.0.0.1"):
    with socket.create_connection((host, port), timeout=3) as s:
        s.settimeout(None)
        for cmd in commands:
            s.sendall((cmd + " \n").encode())
            time.sleep(0.2)  # 等待命令执行
        # 可选：读取返回内容
        # data = s.recv(4096)
        # print(data.decode(errors="ignore"))

def start_openocd(cfg_path):
    # Windows 下用 shell=True 以便于查找 openocd.exe
    proc = subprocess.Popen(
        ["openocd", "-f", cfg_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
        bufsize=1,
        universal_newlines=True
    )
    # 实时读取并染色 stderr
    def color_stderr():
        for line in proc.stderr:
            # Windows 控制台支持 ANSI，PowerShell 5.1 需开启 VT100
            sys.stderr.write(f"\033[31m{line.rstrip()}\033[0m\n")
            sys.stderr.flush()
    t = threading.Thread(target=color_stderr, daemon=True)
    t.start()
    # 等待 openocd 启动并监听端口
    for _ in range(30):
        if is_port_open(OPENOCD_PORT):
            return proc
        time.sleep(1)
    proc.terminate()
    raise RuntimeError("OpenOCD 启动失败，端口未监听")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    rtt_path = os.path.join(script_dir, "rtt.cfg")

    parser = argparse.ArgumentParser(description="OpenOCD RTT 启动与连接脚本")
    parser.add_argument("cfg", help="openocd 配置文件路径", default="openocd.cfg", nargs="?")
    args = parser.parse_args()

    openocd_started = is_port_open(OPENOCD_PORT)
    proc = None
    if not openocd_started:
        print("OpenOCD 未启动，正在启动...")
        proc = start_openocd(args.cfg)
    else:
        print("OpenOCD 已在运行，直接连接")

    print("配置 RTT...")    
    send_commands(OPENOCD_PORT, [f"source [find {rtt_path.replace(os.sep, '/')}]" ])

    time.sleep(1)

    try:
        print("尝试连接 RTT 端口，输入 Ctrl+C 退出")
        with socket.create_connection(("127.0.0.1", RTT_PORT), timeout=2) as s:
            s.setblocking(True)
            while True:
                data = s.recv(4096)
                if not data:
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n已退出 RTT 连接")
    
    while proc and is_port_open(OPENOCD_PORT):
        print("尝试停止 OpenOCD 进程")
        try:
            os.kill(proc.pid, signal.SIGINT)
        except:
            pass

if __name__ == "__main__":
    main()

