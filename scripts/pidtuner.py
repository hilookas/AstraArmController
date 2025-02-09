import tkinter as tk
from tkinter import ttk
import threading
import time
import math
import struct
import serial
import socket

class PIDTuner:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("PID Tuner")

        # 设置初始PID值
        self.p_val = tk.DoubleVar(value=10.0)
        self.i_val = tk.DoubleVar(value=7.0)
        self.d_val = tk.DoubleVar(value=20.0)
        self.i_clip_thres_val = tk.DoubleVar(value=10.0)
        self.i_clip_coef_val = tk.DoubleVar(value=0.5)

        # 创建P滑块
        self.create_slider("P", self.p_val, 0, 80, 0)
        
        # 创建I滑块
        self.create_slider("I", self.i_val, 0, 10, 1)
        
        # 创建D滑块
        self.create_slider("D", self.d_val, 0, 80, 2)
        
        self.create_slider("i_clip_thres", self.i_clip_thres_val, 0, 100, 3)
        self.create_slider("i_clip_coef", self.i_clip_coef_val, 0, 1, 4)

        # 显示PID值标签
        self.label = tk.Label(self.root, text="")
        self.update_label()
        self.label.grid(row=5, column=0, columnspan=2, pady=10)

        # 绑定更新事件
        self.p_val.trace("w", self.update_label)
        self.i_val.trace("w", self.update_label)
        self.d_val.trace("w", self.update_label)
        self.i_clip_thres_val.trace("w", self.update_label)
        self.i_clip_coef_val.trace("w", self.update_label)
        
        self.updated = True

    def create_slider(self, name, variable, min_val, max_val, row):
        """创建一个滑块"""
        label = tk.Label(self.root, text=f"{name} Gain")
        label.grid(row=row, column=0, padx=10, pady=5)

        slider = ttk.Scale(
            self.root, from_=min_val, to=max_val, orient="horizontal", variable=variable, length=300
        )
        slider.grid(row=row, column=1, padx=10, pady=5)

    def update_label(self, *args):
        """更新PID显示标签"""
        self.p = self.p_val.get()
        self.i = self.i_val.get()
        self.d = self.d_val.get()
        self.i_clip_thres = self.i_clip_thres_val.get()
        self.i_clip_coef = self.i_clip_coef_val.get()
        self.label.config(text=f"P: {self.p:.2f}, I: {self.i:.2f}, D: {self.d:.2f}, i_clip_thres: {self.i_clip_thres:.2f}, i_clip_coef: {self.i_clip_coef:.2f}")
        
        self.updated = True

class PIDTunerRunner:
    def __init__(self):
        self.app = None
        # 创建并启动GUI线程
        gui_thread = threading.Thread(target=self.run_gui)
        gui_thread.daemon = True  # 守护线程，主线程结束时自动退出
        gui_thread.start()

    def run_gui(self):
        """运行Tkinter GUI的线程"""
        self.app = PIDTuner()
        self.app.root.mainloop()

def generator_sweep_wave(sample_T):
    mn = int(0x0800 - 0x300)
    mx = int(0x0800 + 0x300)
    t = 0
    T = 2 # s
    
    for vel in range(50, 2000, 100):     
        print(vel)
        
        value = mn
        while value < mx:
            value += sample_T * vel
            # print(int(t * 1000), value)
            yield int(value)
            t += sample_T
        
        value = mx
        while value > mn:
            value -= sample_T * vel
            # print(int(t * 1000), value)
            yield int(value)
            t += sample_T

def generator_sin_wave(sample_T):
    mn = int(0x0800 - 0x200 * 0.5)
    mx = int(0x0800 + 0x200 * 0.5)
    t = 0
    T = 2 # s

    while True:
        value = int((math.sin(2 * math.pi * t / T) + 1) / 2 * (mx - mn) + mn)
        # value = mx if math.sin(2 * math.pi * t / T) > 0 else mn
        print(int(t * 1000), value)
        
        yield value
        t += sample_T

def generator_square_wave(sample_T):
    mn = int(0x0800 - 0x200 * 0.5)
    mx = int(0x0800 + 0x200 * 0.5)
    
    T = 4 # s
    
    while True:
        t = 0

        while t < T / 2:
            value = mn
            yield value
            t += sample_T

        while t < T:
            value = mx
            yield value
            t += sample_T

if __name__ == '__main__':
    COMM_HEAD = 0x5A
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_PIDTUNE = 0x08
    COMM_TYPE_TORQUE = 0x04

    ser = serial.Serial("/dev/tty_puppet_right", 921600, timeout=None)
    fd = open('data.txt', 'wb')

    def plot_thread():
        # use vofa
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

        while True:
            data = ser.read()
            sock.sendto(data, ("192.168.1.14", 10086))
            # sock.sendto(bytes(f"{11111}\n", "utf-8"), ("127.0.0.1", 10086))
            fd.write(data)

    threading.Thread(target=plot_thread, daemon=True).start()
    
    pidtuner = PIDTunerRunner()

    sample_T = 0.02
    
    # g = generator_sin_wave(sample_T)
    g = generator_square_wave(sample_T)
    
    try:
        while True:
            if pidtuner.app and pidtuner.app.updated:
                p = pidtuner.app.p
                i = pidtuner.app.i
                d = pidtuner.app.d
                i_clip_thres = pidtuner.app.i_clip_thres
                i_clip_coef = pidtuner.app.i_clip_coef
                # Racing condition
                print(f"Updated: P: {p:.2f}, I: {i:.2f}, D: {d:.2f}, i_clip_thres: {i_clip_thres:.2f}, i_clip_coef: {i_clip_coef:.2f}")
                pidtuner.app.updated = False
                ser.write(struct.pack('>BBffffffff', COMM_HEAD, COMM_TYPE_PIDTUNE, *[p, i, d, i_clip_thres, i_clip_coef, 0, 0, 0]))
        
            value = next(g)
        
            # ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, 0x0C00, 0x0800, 0x0800, 0x0800, 0x0800])) # 1 joint
            # ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, value, 0x0800, 0x0800, 0x0800, 0x0800])) # 2 joint
            ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800]))
            
            time.sleep(sample_T)
    
    except KeyboardInterrupt:
        ser.write(struct.pack('>BBBxxxxxxxxxxxxxxx', COMM_HEAD, COMM_TYPE_TORQUE, 0))
        ser.flush()
        ser.close()
        fd.close()