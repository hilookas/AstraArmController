import matplotlib.pyplot as plt
import math
import numpy as np

if __name__ == '__main__':
    # save raw_goal_pos  smoothed_goal_pos real_pos timestamps
    raw_goal_pos, smoothed_goal_pos, real_pos, timestamps = np.load(f'experiment_result_square/position_data.npy')
    raw_goal_pos_disabled, smoothed_goal_pos_disabled, real_pos_disabled, timestamps_disabled = np.load(f'experiment_result_backlash_compensate_0_square/position_data.npy')
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(timestamps, raw_goal_pos, '--', label='Goal')
    ax1.plot(timestamps_disabled, real_pos_disabled, label='Disabled')
    ax1.plot(timestamps, real_pos, label='Enabled')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (deg)")
    # ax1.set_title("Square Wave Result with Backlash Compensation")
    ax1.legend(["Goal", "Disabled", "Enabled"], loc='upper left')
    fig1.savefig(f"backlash_squarewave_position.pdf")
    # half height 
    fig1.set_size_inches(fig1.get_size_inches() * 0.5)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(timestamps_disabled, real_pos_disabled - smoothed_goal_pos_disabled, label='Disabled')
    ax2.plot(timestamps, real_pos - smoothed_goal_pos, label='Enabled')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position Error (deg)")
    # ax2.set_title("Square Wave Result with Backlash Compensation")
    ax2.legend(["Disabled", "Enabled"], loc='upper left')
    fig2.savefig(f"backlash_squarewave_error.pdf")
    
    
    
    # save raw_goal_pos  smoothed_goal_pos real_pos timestamps
    raw_goal_pos, smoothed_goal_pos, real_pos, timestamps = np.load(f'experiment_result_sine/position_data.npy')
    raw_goal_pos_disabled, smoothed_goal_pos_disabled, real_pos_disabled, timestamps_disabled = np.load(f'experiment_result_backlash_compensate_0_sine/position_data.npy')
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(timestamps, raw_goal_pos, '--', label='Goal')
    ax1.plot(timestamps_disabled, real_pos_disabled, label='Disabled')
    ax1.plot(timestamps, real_pos, label='Enabled')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (deg)")
    # ax1.set_title("Square Wave Result with Backlash Compensation")
    ax1.legend(["Goal", "Disabled", "Enabled"], loc='upper right')
    fig1.savefig(f"backlash_sinewave_position.pdf")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(timestamps_disabled, real_pos_disabled - smoothed_goal_pos_disabled, label='Disabled')
    ax2.plot(timestamps, real_pos - smoothed_goal_pos, label='Enabled')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position Error (deg)")
    # ax2.set_title("Square Wave Result with Backlash Compensation")
    ax2.legend(["Disabled", "Enabled"], loc='upper right')
    fig2.savefig(f"backlash_sinewave_error.pdf")



    # save raw_goal_pos  smoothed_goal_pos real_pos timestamps
    raw_goal_pos, smoothed_goal_pos, real_pos, timestamps = np.load(f'experiment_result_stair/position_data.npy')
    raw_goal_pos_disabled, smoothed_goal_pos_disabled, real_pos_disabled, timestamps_disabled = np.load(f'experiment_result_sticktion_compensation0_stair/position_data.npy')
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(timestamps, raw_goal_pos, '--', label='Goal')
    ax1.plot(timestamps_disabled, real_pos_disabled, label='Disabled')
    ax1.plot(timestamps, real_pos, label='Enabled')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (deg)")
    # ax1.set_title("Square Wave Result with Backlash Compensation")
    ax1.legend(["Goal", "Disabled", "Enabled"], loc='upper left')
    fig1.savefig(f"stiction_stairwave_position.pdf")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(timestamps_disabled, real_pos_disabled - smoothed_goal_pos_disabled, label='Disabled')
    ax2.plot(timestamps, real_pos - smoothed_goal_pos, label='Enabled')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position Error (deg)")
    # ax2.set_title("Square Wave Result with Backlash Compensation")
    ax2.legend(["Disabled", "Enabled"], loc='upper left')
    fig2.savefig(f"stiction_stairwave_error.pdf")
