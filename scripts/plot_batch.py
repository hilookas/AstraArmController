import matplotlib.pyplot as plt
import math
import numpy as np
import glob

if __name__ == '__main__':
    for experiment_directory in glob.glob('squarewave/experiment_result*'):
        timestamps = []
        raw_goal_pos = []
        smoothed_goal_pos = []
        real_pos = []
        
        with open(f'{experiment_directory}/data.txt', 'r') as f:
            lines = f.readlines()
            for line in lines:
                if ',' in line:
                    data = [float(x.strip()) for x in line.split(',')]
                    
                    raw_goal_pos.append(data[0])
                    smoothed_goal_pos.append(data[2])
                    real_pos.append(data[4])
                    timestamps.append(int(data[-1]))
        
        raw_goal_pos = (np.array(raw_goal_pos, dtype=np.float32) - 2048) / 4096 * 360
        smoothed_goal_pos = (np.array(smoothed_goal_pos, dtype=np.float32) - 2048) / 4096 * 360
        real_pos = (np.array(real_pos, dtype=np.float32) - 2048) / 4096 * 360
        timestamps = np.array(timestamps, dtype=np.float32) / 1000
        
        # import IPython; IPython.embed()
        
        # filter out the leading and tail data with 1.5s
        valid_idx = np.argwhere((timestamps[0] + 1.5 < timestamps) & (timestamps < timestamps[-1] - 1.5))
        timestamps = timestamps[valid_idx]
        raw_goal_pos = raw_goal_pos[valid_idx]
        smoothed_goal_pos = smoothed_goal_pos[valid_idx]
        real_pos = real_pos[valid_idx]
        
        timestamps -= timestamps[0]

        fig3 = plt.figure()
        ax3 = fig3.add_subplot(111)
        ax3.set_ylim(-5, 5)
        ax3.plot(timestamps, (real_pos - smoothed_goal_pos) / 360 * 4096)
        ax3.plot(timestamps, np.zeros_like(timestamps), 'k--')
        ax3.set_xlabel("Timestamps")
        ax3.set_ylabel("Position Error (4096 counts)")
        # ax3.set_title("Position Error")
        ax3.legend(["Position Error"], loc='upper right')
        fig3.savefig(f"{experiment_directory}/position_error2.pdf")
