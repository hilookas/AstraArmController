import socket
import serial
import struct
import threading
import time
import math

from pidtuner import PIDTunerRunner

def generator(sample_T):
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
        

if __name__ == '__main__':
    COMM_HEAD = 0x5A
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_PIDTUNE = 0x08

    ser = serial.Serial("COM4", 921600, timeout=None)
    fd = open('data.txt', 'wb')

    def plot_thread():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

        while True:
            data = ser.read()
            sock.sendto(data, ("127.0.0.1", 10086))
            # sock.sendto(bytes(f"{11111}\n", "utf-8"), ("127.0.0.1", 10086))
            fd.write(data)

    threading.Thread(target=plot_thread, daemon=True).start()
    
    pidtuner = PIDTunerRunner()

    sample_T = 0.02
    
    g = generator(sample_T)
    
    while True:
        if pidtuner.app and pidtuner.app.updated:
            p = pidtuner.app.p
            i = pidtuner.app.i
            d = pidtuner.app.d
            # Racing condition
            print(f"Updated: P: {p:.2f}, I: {i:.2f}, D: {d:.2f}")
            pidtuner.app.updated = False
            ser.write(struct.pack('>BBffffffff', COMM_HEAD, COMM_TYPE_PIDTUNE, *[p, i, d, 0, 0, 0, 0, 0]))
    
        value = next(g)
    
        ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, 0x0C00, 0x0800, 0x0800, 0x0800, 0x0800]))
        
        time.sleep(sample_T)