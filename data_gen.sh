#!/bin/bash

# Section 3.2: Goal-directed Discrete Movement: Joint-space
python3 -B main_joint_trajectory_discrete.py --start_time 0.3 --run_time 4.0 --save_data --record_vid --sim_type motor 
python3 -B main_joint_trajectory_discrete.py --start_time 0.3 --run_time 4.0 --save_data --record_vid --sim_type movement

# # Section 3.3: Goal-directed Discrete Movement: Task-space
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 0 --record_vid --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 1 --record_vid --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 2 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 3 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 4 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 5 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 6 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 7 --record_vid  --save_data --sim_type motor

python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 0 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 1 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 2 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 3 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 4 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 5 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 6 --record_vid  --save_data --sim_type movement
python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 7 --record_vid  --save_data --sim_type movement

# Section 3.4: Managing Kinematic Redundancy
python3 -B main_task_trajectory_redund.py --start_time 0.3 --run_time 4.0 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_redund.py --start_time 0.3 --run_time 4.0 --record_vid  --save_data --sim_type movement

# Section 3.5: Sequenced Discrete Movements
python3 -B main_sequence_movement.py --start_time 0.3 --run_time 4.0 --record_vid  --save_data --sim_type motor
python3 -B main_sequence_movement.py --start_time 0.3 --run_time 4.0 --record_vid  --save_data --sim_type movement

# Section 3.6: Rhythmic Movement
python3 -B main_task_trajectory_rhythmic.py --run_time 16.0 --record_vid  --save_data --sim_type motor
python3 -B main_task_trajectory_rhythmic.py --run_time 16.0 --record_vid  --save_data --sim_type movement

# Section 3.7: Combination of Discrete and Rhythmic Movements
python3 -B main_discrete_and_rhythmic.py --run_time 20.0 --record_vid  --save_data --sim_type motor
python3 -B main_discrete_and_rhythmic.py --run_time 20.0 --record_vid  --save_data --sim_type movement

# Section 3.8: Obstacle Avoidance
python3 -B main_obstacle_avoidance.py --run_time 10.0 --record_vid  --save_data --sim_type motor
python3 -B main_obstacle_avoidance.py --run_time 10.0 --record_vid  --save_data --sim_type movement
