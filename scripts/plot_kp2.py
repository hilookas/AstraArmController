import matplotlib.pyplot as plt
import numpy as np

kp = 30
kp2 = 10
kp2_err_point = 10

errs = np.linspace(-100, 100, 100)
outs = []

for err in errs:
    outs.append(kp * err + (kp2 - kp) * (err - kp2_err_point if err > kp2_err_point else 0) + (kp2 - kp) * (err + kp2_err_point if err < -kp2_err_point else 0))

plt.plot(errs, outs)
plt.xlabel('Error')
plt.ylabel('Output')
plt.savefig('kp2_demo.pdf')