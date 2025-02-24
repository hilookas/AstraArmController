import tkinter as tk
from tkinter import ttk
import threading
import time
import math
import struct
import serial
import socket
import os

def generator_sine_wave(sample_T):
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    mn = int(0x0800 - 0x200 * 0.5)
    mx = int(0x0800 + 0x200 * 0.5)
    T = 3 # s
    
    for i in range(3):
        t = 0
        while t < T:
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
    T = 4 # s

    for i in range(3):
        t = 0
        while t < T / 2:
            value = mn
            yield value
            t += sample_T
        while t < T:
            value = mx
            yield value
            t += sample_T
        
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    yield None
    
def generator_stair_wave(sample_T):
    t = 0
    while t < 2:
        yield int(0x0800)
        t += sample_T
    
    value = 0x0800
    step = 0x0002
    T = 2 # s
    
    for i in range(4):
        value += step

        t = 0
        while t < T:
            yield value
            t += sample_T

    for i in range(4):
        value -= step

        t = 0
        while t < T:
            yield value
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
    
    # ser.write(struct.pack('>BBBBBBBBBBBBBBBBBB', COMM_HEAD, 0x04, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))
    # ser.flush()
    # exit()

    p = 30
    i = 0
    d = 0
    p2 = 10
    p2_err_point = 2
    i_max = 800.0
    i_clip_thres = 10.0
    i_clip_coef = 0.5
    # Racing condition
    print(f"Updated: P: {p:.2f}, I: {i:.2f}, D: {d:.2f}, p2: {p2:.2f}, p2_err_point: {p2_err_point:.2f}, i_max: {i_max:.2f}, i_clip_thres: {i_clip_thres:.2f}, i_clip_coef: {i_clip_coef:.2f}")
    ser.write(struct.pack('>BBffffffff', COMM_HEAD, COMM_TYPE_PIDTUNE, *[p, i, d, i_clip_thres, i_clip_coef, i_max, p2, p2_err_point]))
    
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
        fd.flush()
        fd.close()