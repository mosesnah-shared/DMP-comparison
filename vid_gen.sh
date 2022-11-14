#!/bin/bash

# python3 -B main_joint_trajectory.py --start_time 1.0 --run_time 5.0 --record_vid --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 0 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 1 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 2 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 3 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 4 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 5 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 6 --save_data
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 2.0 --target_idx 7 --save_data

python3 -B main_task_trajectory_redund.py --start_time 1.0 --run_time 6.0 #--save_data --record_vid

# python3 -B main_sequence_movement.py --start_time 1.0 --run_time 5.0 --record_vid