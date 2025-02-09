import matplotlib.pyplot as plt
import math
import numpy as np

if __name__ == '__main__':
    experiment_directory = 'experiment_result_sinewave_backlash_disabled_aggressive_pid'
    
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
    
    # save raw_goal_pos  smoothed_goal_pos real_pos timestamps
    np.save(f'{experiment_directory}/position_data.npy', [raw_goal_pos, smoothed_goal_pos, real_pos, timestamps])
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(timestamps, raw_goal_pos)
    ax1.plot(timestamps, smoothed_goal_pos)
    ax1.plot(timestamps, real_pos)
    ax1.set_xlabel("Timestamps")
    ax1.set_ylabel("Position (deg)")
    ax1.set_title("Position")
    ax1.legend(["Raw Goal Position", "Smoothed Goal Position", "Real Position"], loc='upper right')
    fig1.savefig(f"{experiment_directory}/position.pdf")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(timestamps, real_pos - smoothed_goal_pos)
    ax2.set_xlabel("Timestamps")
    ax2.set_ylabel("Position Error (deg)")
    ax2.set_title("Position Error")
    ax2.legend(["Position Error"], loc='upper right')
    fig2.savefig(f"{experiment_directory}/position_error.pdf")
