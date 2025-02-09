import matplotlib.pyplot as plt
import math
import numpy as np

if __name__ == '__main__':
    # save raw_goal_pos  smoothed_goal_pos real_pos timestamps
    raw_goal_pos, smoothed_goal_pos, real_pos, timestamps = np.load(f'experiment_result_squarewave_backlash_enabled_aggressive_pid/position_data.npy')
    raw_goal_pos_disabled, smoothed_goal_pos_disabled, real_pos_disabled, timestamps_disabled = np.load(f'experiment_result_squarewave_backlash_disabled_kp_3.0_ki_5.0/position_data.npy')
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(timestamps, real_pos)
    ax1.plot(timestamps_disabled, real_pos_disabled)
    ax1.plot(timestamps, raw_goal_pos, '--')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (deg)")
    ax1.set_title("Square Wave Result with Backlash Compensation")
    ax1.legend(["Enabled", "Disabled", "Goal"], loc='upper right')
    fig1.savefig(f"squarewave_position.pdf")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(timestamps, real_pos - smoothed_goal_pos)
    ax2.plot(timestamps_disabled, real_pos_disabled - smoothed_goal_pos_disabled)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position Error (deg)")
    ax2.set_title("Square Wave Result with Backlash Compensation")
    ax2.legend(["Enabled", "Disabled"], loc='upper right')
    fig2.savefig(f"squarewave_position_error.pdf")
    
    
    # save raw_goal_pos  smoothed_goal_pos real_pos timestamps
    raw_goal_pos, smoothed_goal_pos, real_pos, timestamps = np.load(f'experiment_result_sinewave_backlash_enabled_aggressive_pid/position_data.npy')
    raw_goal_pos_disabled, smoothed_goal_pos_disabled, real_pos_disabled, timestamps_disabled = np.load(f'experiment_result_sinewave_backlash_disabled_kp_3.0_ki_5.0/position_data.npy')
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(timestamps, real_pos)
    ax1.plot(timestamps_disabled, real_pos_disabled)
    ax1.plot(timestamps, raw_goal_pos, '--')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (deg)")
    ax1.set_title("Square Wave Result with Backlash Compensation")
    ax1.legend(["Enabled", "Disabled", "Goal"], loc='upper right')
    fig1.savefig(f"sinewave_position.pdf")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(timestamps, real_pos - smoothed_goal_pos)
    ax2.plot(timestamps_disabled, real_pos_disabled - smoothed_goal_pos_disabled)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position Error (deg)")
    ax2.set_title("Square Wave Result with Backlash Compensation")
    ax2.legend(["Enabled", "Disabled"], loc='upper right')
    fig2.savefig(f"sinewave_position_error.pdf")
    