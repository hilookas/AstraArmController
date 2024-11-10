import socket
import serial
import struct
import threading
import time
import math

from pidtuner import PIDTunerRunner

def generator(sample_T):
    mn = int(0x0800 - 0x200)
    mx = int(0x0800 + 0x200)
    t = 0
    T = 4 # s

    while True:
        # value = int((math.sin(2 * math.pi * t / T) + 1) / 2 * (mx - mn) + mn)
        value = mx if math.sin(2 * math.pi * t / T) > 0 else mn
        print(int(t * 1000), value)
        
        yield value
        t += sample_T
        

if __name__ == '__main__':
    COMM_HEAD = 0x5A
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_PIDTUNE = 0x08

    ser = serial.Serial("COM4", 921600, timeout=None)

    def plot_thread():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

        while True:
            data = ser.read()
            sock.sendto(data, ("127.0.0.1", 10086))
            # sock.sendto(bytes(f"{11111}\n", "utf-8"), ("127.0.0.1", 10086))

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
    
        ser.write(struct.pack('>BBHHHHHHxxxx', COMM_HEAD, COMM_TYPE_CTRL, *[value, value, 0x0800, 0x0800, 0x0800, 0x0800]))
        
        time.sleep(sample_T)