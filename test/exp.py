import socket
import serial
import struct
import threading
import time
import math

if __name__ == '__main__':
    COMM_HEAD = 0x5A
    COMM_TYPE_CTRL = 0x02

    ser = serial.Serial("COM4", 921600, timeout=None)

    def plot_thread():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

        while True:
            data = ser.read()
            sock.sendto(data, ("127.0.0.1", 10086))
            # sock.sendto(bytes(f"{11111}\n", "utf-8"), ("127.0.0.1", 10086))

    threading.Thread(target=plot_thread, daemon=True).start()
    
    min = 0x0800
    max = 0x08F0
    control_T = 0.02
    t = 0
    T = 1 # s
    
    while True:
        value = int((math.sin(2 * math.pi * t / T) + 1) / 2 * (max - min) + min)
        # value = max if math.sin(2 * math.pi * t / T) > 0 else min
        print(int(t * 1000), value)
    
        ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, value, 0x0800, 0x0800, 0x0800, 0x0800]))
        
        time.sleep(control_T)
        t += control_T