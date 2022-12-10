"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, no Kinematic Redundancy
|                 Section 3.3 
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import shutil
import scipy.io
import numpy             as np
import moviepy.editor    as mpy
import matplotlib.pyplot as plt

# Adding local modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

# Adding DMP Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives
from InverseDynamicsModel       import get2DOF_J, get2DOF_M, get2DOF_C, get2DOF_dJ

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( my_sim ):

    # Define the controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "task_imp" )
    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    # The number of actuators
    n = my_sim.n_act

    # Setting the initial conditino of the robots
    q1 = np.pi * 1/4
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial end-effector position, and 8 targets in total
    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    idx = args.target_idx
    xEEf = xEEi + 0.5 * np.array( [ np.cos( idx * np.pi/4 ), np.sin( idx * np.pi/4 ), 0 ] )
    ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 1, ti = args.start_time  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

    # The time step of the simulation 
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    dmp_list = [] 

    # The number of basis functions for the imitation learning
    N = 10

    # The x and y coordinates. 
    tmp_str = [ "x", "y" ]
    for i in range( 2 ):
        dmp = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp" + tmp_str[ i ], cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )
        
    # Setting the initial condition of the arm posture 
    q1 = np.pi * 1/4
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( my_sim.nq ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The parameters of min-jerk-traj
    idx = args.target_idx    
    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    p0f = p0i + 0.5 * np.array( [ np.cos( idx * np.pi/4 ), np.sin( idx * np.pi/4 ), 0 ] )
    D = 1.0     

    # The time constant tau is the duration of the movement. 
    cs.tau = D        

    # The number of sample points for imitation learning
    P = 100

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = D/P

    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 2, P + 1 ) )
    dp_des  = np.zeros( ( 2, P + 1 ) ) 
    ddp_des = np.zeros( ( 2, P + 1 ) )

    for i in range( 2 ):
        for j in range( P + 1 ):
            t = tmp_dt * j
            p_des[ i, j ], dp_des[ i, j ], ddp_des[ i, j ] = min_jerk_traj( t, 0.0, p0i[ i ], p0f[ i ], D  )

        # Learning the trajectory 
        t_arr = tmp_dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ] )


    # Now, we integrate this solution
    # The number of time step for the simulation
    N_sim = round( args.run_time/dt ) + 1
    
    # The p, dp, ddp of the robot 
    # These commands should be later converted to joint space coordinates. 
    p_command   = np.zeros( ( nq, N_sim ) )
    dp_command  = np.zeros( ( nq, N_sim ) )
    ddp_command = np.zeros( ( nq, N_sim ) )

    # Iterating through the dmps
    for i in range( 2 ):

        dmp = dmp_list[ i ]

        # y, z, dy, dz
        t_arr, y_arr, _, dy_arr, dz_arr = dmp.integrate( p0i[ i ], 0, p0f[ i ], dt, N_sim )

        p_command[   i, : ] =   y_arr
        dp_command[  i, : ] =  dy_arr
        ddp_command[ i, : ] =  dz_arr 

    # Since we now know the q_command, looping through the simulation 
    t = 0.
    T = args.run_time 

    n_steps = 0
    frames = [ ]
    
    if args.cam_pos is not None: my_sim.set_camera_pos( ) 

    # Saving the data if on
    if args.is_save_data:
        q_arr   = [ ]
        dq_arr  = [ ]
        ddq_arr = [ ]

        p_arr   = [ ]
        dp_arr  = [ ]
        tau_arr = [ ]
        
        t_sim_arr = [ ]

        # From the inverse kinematics of the robot
        # This will be different with the actual q trajectory of the robot
        q_command_arr  = [ ]
        dq_command_arr = [ ]
        ddq_command_arr = [ ]

    while t <= T + 1e-7:

        # Render the simulation if mj_viewer exists        
        if my_sim.mj_viewer is not None and n_steps % my_sim.vid_step == 0:

            # Render the simulation if mj_viewer exists        
            my_sim.mj_viewer.render( )

            if args.is_record_vid: 
                # Read the raw rgb image
                rgb_img = my_sim.mj_viewer.read_pixels( my_sim.mj_viewer.width, my_sim.mj_viewer.height, depth = False )

                # Convert BGR to RGB and flip upside down.
                rgb_img = np.flip( rgb_img, axis = 0 )
                
                # Add the frame list, this list will be converted to a video via moviepy library
                frames.append( rgb_img )  

            # If reset button (BACKSPACE) is pressed
            if my_sim.mj_viewer.is_reset: my_sim.mj_viewer.is_reset = False
            
            # If SPACE BUTTON is pressed
            if my_sim.mj_viewer.is_paused:    continue

        px = p_command[ 0, n_steps ]
        py = p_command[ 1, n_steps ]
        
        # Solve the inverse kinematics 
        q2 = np.pi - np.arccos( 0.5 * ( 2 - px ** 2 - py ** 2  ) )
        q1 = np.arctan2( py, px ) - q2/2 

        # The joint trajectories
        q   = np.array( [ q1, q2 ] )
        dq  = np.linalg.inv( get2DOF_J( q ) ) @ dp_command[ :, n_steps ]
        ddq = np.linalg.inv( get2DOF_J( q ) ) @ ( ddp_command[ :, n_steps ] - get2DOF_dJ( q, dq  ) @ dq)

        # Calculate the mass, coriolis matrix
        tau = get2DOF_M( q ) @ ddq + get2DOF_C( q, dq ) @ dq

        my_sim.mj_data.ctrl[ :my_sim.n_act ] = tau

        if args.is_save_data:

            t_sim_arr.append( t )

            q_command_arr.append(     q )
            dq_command_arr.append(   dq )
            ddq_command_arr.append( ddq )

            q_arr.append(   np.copy( my_sim.mj_data.qpos[ : ] ) )
            dq_arr.append(  np.copy( my_sim.mj_data.qvel[ : ] ) )
            ddq_arr.append( np.copy( my_sim.mj_data.qacc[ : ] ) )

            p_arr.append(   np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) )
            dp_arr.append(  np.copy( my_sim.mj_data.get_site_xvelp( "site_end_effector" ) ) )
        
            tau_arr.append( tau )

        my_sim.step( )
        n_steps += 1
        t += dt

    # If video should be recorded, write the video file. 
    if args.is_record_vid and frames is not None:
        clip = mpy.ImageSequenceClip( frames, fps = my_sim.fps )
        clip.write_videofile( my_sim.tmp_dir + "video.mp4", fps = my_sim.fps, logger = None )

    # If video recorded/save data is true, then copy the model, main file and the arguments passed
    if args.is_record_vid or args.is_save_data:
        shutil.copyfile( C.MODEL_DIR + my_sim.model_name + ".xml", my_sim.tmp_dir + "model.xml" )                    
    
    # Saving the data for analysis
    if args.is_save_data:
        
        # Packing up the arrays as a dictionary
        # DMP for the first joint
        for i in range( nq ):
            dmp_list[ i ].save_mat_data( my_sim.tmp_dir )

        # Saving the simulation data
        dict  = { "t_sim_arr": t_sim_arr, "q_arr": q_arr, "dq_arr": dq_arr, "ddq_arr": ddq_arr,
                  "p_arr": p_arr, "dq_arr":dp_arr,  "tau_arr": tau_arr, 
                  "q_command_arr": q_command_arr, "dq_command_arr": dq_command_arr, "ddq_command_arr": ddq_command_arr } 

        scipy.io.savemat( my_sim.tmp_dir + "/dmp_sim.mat", { **dict } )    

    # Move the tmp folder to results if not empty, else just remove the tmp file. 
    shutil.move( my_sim.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( my_sim.tmp_dir ) ) != 0 else os.rmdir( my_sim.tmp_dir )


if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    ctrl_type = "movement"
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "2DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 1, 0, 5, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

