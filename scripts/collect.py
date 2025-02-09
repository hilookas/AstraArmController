import tkinter as tk
from tkinter import ttk
import threading
import time
import math
import struct
import serial
import socket
import os

def generator_sin_wave(sample_T):
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    mn = int(0x0800 - 0x200 * 0.5)
    mx = int(0x0800 + 0x200 * 0.5)
    t = 0
    T = 2 # s

    while t < T * 3:
        value = int((math.sin(2 * math.pi * t / T) + 1) / 2 * (mx - mn) + mn)
        yield value
        t += sample_T
        
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    yield None

def generator_square_wave(sample_T):
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    mn = int(0x0800 - 0x200 * 0.5)
    mx = int(0x0800 + 0x200 * 0.5)
    t = 0
    T = 3 # s

    while t < T * 3:
        t_ = 0

        while t_ < T / 2:
            value = mn
            yield value
            t_ += sample_T
            t += sample_T

        while t_ < T:
            value = mx
            yield value
            t_ += sample_T
            t += sample_T
        
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    yield None


if __name__ == '__main__':
    COMM_HEAD = 0x5A
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_PIDTUNE = 0x08
    COMM_TYPE_TORQUE = 0x04
    
    os.makedirs('experiment_result', exist_ok=True)

    ser = serial.Serial("/dev/tty_puppet_right", 921600, timeout=None)
    fd = open('experiment_result/data.txt', 'wb')

    # p = 3.0
    # i = 5.0
    p = 10.0
    i = 7.0
    d = 20.0
    i_clip_thres = 10.0
    i_clip_coef = 0.5
    # Racing condition
    print(f"Updated: P: {p:.2f}, I: {i:.2f}, D: {d:.2f}, i_clip_thres: {i_clip_thres:.2f}, i_clip_coef: {i_clip_coef:.2f}")
    ser.write(struct.pack('>BBffffffff', COMM_HEAD, COMM_TYPE_PIDTUNE, *[p, i, d, i_clip_thres, i_clip_coef, 0, 0, 0]))
    
    try:
        def plot_thread():
            # use vofa
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

            while True:
                data = ser.read()
                sock.sendto(data, ("192.168.1.14", 10086))
                # sock.sendto(bytes(f"{11111}\n", "utf-8"), ("127.0.0.1", 10086))
                fd.write(data)

        threading.Thread(target=plot_thread, daemon=True).start()
        
        sample_T = 0.02
        
        g = generator_square_wave(sample_T)

        while True:
            value = next(g)
            if value is None:
                break
        
            ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800]))
            
            time.sleep(sample_T)
    
    except KeyboardInterrupt:
        pass

    finally:
        ser.write(struct.pack('>BBBxxxxxxxxxxxxxxx', COMM_HEAD, COMM_TYPE_TORQUE, 0))
        ser.flush()
        ser.close()
        fd.close()