import matplotlib.pyplot as plt
import numpy as np

TIMER_TIMEOUT_US = 15000

labels = [
    "raw_goal_pos", "raw_goal_pos2",  # 前馈位置（命令）
    "goal_pos", "goal_pos2", # 前馈位置（加减速规划后）
    "last_pos", "last_pos2", # 反馈位置（编码器读数）
    "debug_signal", "debug_signal2", 
    "last_vel", "last_vel2", # 差分速度得反馈速度
    "out", "out2"
]

datas = []
    
with open("data.csv", "r") as f:
    last_pos = None
    last_vel = 0.0
    for line in f:
        df = [float(num.strip()) for num in line.split(",")]
        if last_pos:
            vel = (df[2] - last_pos) / (TIMER_TIMEOUT_US / 1000000)
        else:
            vel = 0.0
        acc = (vel - last_vel) / (TIMER_TIMEOUT_US / 1000000)
        last_pos = df[2]
        last_vel = vel
        
        if abs(acc) < 100:
            df.extend([vel, acc])
            datas.append(df)

# print(datas)
datas = np.array(datas)

# plt.plot(datas[:,13]) # 前馈速度

# plt.hist(abs(datas[:,14]), bins=100) # 前馈加速度

# plt.scatter(datas[:,13], datas[:,10]) # 前馈速度-扭矩
# plt.scatter(datas[:,8], datas[:,10]) # 反馈速度-扭矩

vels = []
tors = []
for vel, tor in zip(datas[:,13], datas[:,10]): # 前馈速度-扭矩
    if vel < 0:
        vel = -vel
        tor = -tor
    vels.append(vel)
    tors.append(tor)

fit_poly = np.polyfit(vels, tors, 1)
print(fit_poly) # [ 0.30385482 61.1004804 ]

plt.scatter(vels, tors, c='none', marker='o', edgecolors='b')
plt.plot(range(0, 1500, 1), np.polyval(fit_poly, range(0, 1500, 1)), 'r')
plt.plot(range(0, 1500, 1), np.polyval(fit_poly, range(0, 1500, 1)) + 60, 'r', linestyle='--')
plt.plot(range(0, 1500, 1), np.polyval(fit_poly, range(0, 1500, 1)) - 60, 'r', linestyle='--')

plt.show()