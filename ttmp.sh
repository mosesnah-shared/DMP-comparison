#!/bin/bash

# python3 -B main_joint_trajectory_discrete.py --start_time 0.0 --run_time 3.0 --save_data --record_vid --sim_type motor 
# python3 -B main_joint_trajectory_discrete.py --start_time 0.0 --run_time 3.0 --save_data --record_vid --sim_type movement

python3 -B main_discrete_and_rhythmic_task.py --start_time 0.0 --run_time 12.0 --sim_type motor # --save_data --record_vid 
# python3 -B main_discrete_and_rhythmic_joint.py --start_time 0.0 --run_time 24.0 --save_data --record_vid --sim_type movement

# python3 -B main_joint_trajectory_discrete.py --start_time 0.0 --run_time 3.0 --save_data --record_vid --sim_type motor 
# python3 -B main_joint_trajectory_discrete.py --start_time 0.0 --run_time 3.0 --save_data --record_vid --sim_type movement

# python3 -B main_discrete_and_rhythmic_joint.py --run_time 20.0 --record_vid  --save_data --sim_type motor
# python3 -B main_discrete_and_rhythmic.py --run_time 20.0 --record_vid  --save_data --sim_type movement

