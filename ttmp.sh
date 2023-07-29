#!/bin/bash

# Section: Task-space Discrete Movement
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 4.0 --target_idx 0 --sim_type motor 	--record_vid --save_data 
# python3 -B main_task_trajectory_no_redund.py --start_time 0.0 --run_time 1.9 --target_idx 0 --sim_type movement --record_vid --save_data 

# python3 -B main_task_trajectory_rhythmic_task.py --start_time 0.0 --run_time 16.0 --sim_type motor 	  --record_vid --save_data 
# python3 -B main_task_trajectory_rhythmic_task.py --start_time 0.0 --run_time 16.0 --sim_type movement --record_vid --save_data 

# python3 -B main_task_trajectory_rhythmic_joint.py --start_time 0.0 --run_time 4.0  --sim_type motor    --record_vid --save_data 
# python3 -B main_task_trajectory_rhythmic_joint.py --start_time 0.0 --run_time 4.0  --sim_type movement --record_vid --save_data 

# Section 3.4: Managing Kinematic Redundancy
# python3 -B main_task_trajectory_redund.py --start_time 0.0 --run_time 4.0 --record_vid  --save_data --sim_type motor
# python3 -B main_task_trajectory_redund.py --start_time 0.0 --run_time 4.0 --record_vid  --save_data --sim_type movement

# Section 3.9: Unexpected Collision
# python3 -B main_unexpected_collision.py --start_time 0.0 --run_time 8.0 --sim_type motor --record_vid --save_data
# python3 -B main_unexpected_collision.py --start_time 0.0 --run_time 4.0 --sim_type movement #--record_vid  --save_data 