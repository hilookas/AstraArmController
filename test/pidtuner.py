import tkinter as tk
from tkinter import ttk
import threading
import time

class PIDTuner:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("PID Tuner")

        # 设置初始PID值
        self.p_val = tk.DoubleVar(value=10.0)
        self.i_val = tk.DoubleVar(value=0.0)
        self.d_val = tk.DoubleVar(value=30.0)

        # 创建P滑块
        self.create_slider("P", self.p_val, 0, 80, 0)
        
        # 创建I滑块
        self.create_slider("I", self.i_val, 0, 10, 1)
        
        # 创建D滑块
        self.create_slider("D", self.d_val, 0, 80, 2)

        # 显示PID值标签
        self.label = tk.Label(self.root, text="")
        self.update_label()
        self.label.grid(row=3, column=0, columnspan=2, pady=10)

        # 绑定更新事件
        self.p_val.trace("w", self.update_label)
        self.i_val.trace("w", self.update_label)
        self.d_val.trace("w", self.update_label)
        
        self.updated = False

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
        self.label.config(text=f"P: {self.p:.2f}, I: {self.i:.2f}, D: {self.d:.2f}")
        
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

if __name__ == "__main__":
    pidtuner = PIDTunerRunner()

    # 主线程可以执行其他任务
    while True:
        # 这里可以放主线程的其他任务代码
        print("Main thread running other tasks...")
        if pidtuner.app and pidtuner.app.updated:
            # Racing condition
            print(f"P: {pidtuner.app.p:.2f}, I: {pidtuner.app.i:.2f}, D: {pidtuner.app.d:.2f}")
            pidtuner.app.updated = False
        time.sleep(2)  # 模拟其他任务的执行