"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, no Kinematic Redundancy
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import math
import shutil
import numpy             as np
import matplotlib.pyplot as plt
import scipy.io
from datetime  import datetime
import moviepy.editor  as mpy


# ======================================================================== #
# ======================================================================== #
#                                                                          #
#                            ADDING LOCAL MODULES                          #
#                                                                          #
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

# ======================================================================== #
# ======================================================================== #
#                                                                          #
#               ADDING MODULES FOR DYNAMIC MOVEMENT PRIMITIVES             #
#                                                                          #
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
 
                                                                                

def run_motor_primitives( my_sim ):

    args = my_sim.args

    # Set initial position
    q0 = np.pi/12
    init_cond = { "qpos": np.array( [ q0, np.pi - 2*q0 ] ),  "qvel": np.zeros( 2 ) }

    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The initial and final posture of the end effector
    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    p0f = p0i + np.array( [ -0.7, 0.7, 0. ] )

    Kx = 300 * np.eye( 3 )
    Bx = 100 * np.eye( 3 )

    # The movement durations of the submovements
    D1, D2 = 1, 1.0

    # Redefine the new time step 
    is_new_goal = False
    g_new = p0f + np.array( [ 1.5, 0.5, 0. ] )

    # Since we now know the q_command, looping through the simulation 
    # We assume pure position control 
    t     = 0.
    dt    = my_sim.mj_model.opt.timestep      
    n_steps = 0 
    T     = args.run_time 
    frames = [ ]    

    q_arr = []
    dq_arr = []
    p_arr = []
    dp_arr = []
    p0_arr = []
    dp0_arr = []

    t_arr = []

    if args.cam_pos is not None: my_sim.set_camera_pos( )     

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
            if my_sim.mj_viewer.is_reset:
                my_sim.mj_viewer.is_reset = False
            
            # If SPACE BUTTON is pressed
            if my_sim.mj_viewer.is_paused:    continue


        # Get the Jacobian of the end-effector
        # The Jacobian is 3-by-nq, although we only need the first two components
        J =  np.copy( my_sim.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )

        # Get the end-effector trajectories
        xEE  = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) )
        dxEE = np.copy( my_sim.mj_data.get_site_xvelp( "site_end_effector" ) )

        t_arr.append( t )
        p_arr.append( xEE )
        q_arr.append(  np.copy( my_sim.mj_data.qpos[ : ] ) )
        
        dp_arr.append( dxEE )
        dq_arr.append( np.copy( my_sim.mj_data.qvel[ : ] ) )        
 
        # The zero-force traejctory (3D)
        x0  = np.zeros( 3 )
        dx0 = np.zeros( 3 )

        for j in range( 3 ):
            tmp_x0, tmp_dx0, _ = min_jerk_traj( t, args.start_time, p0i[ j ], p0f[ j ], D1 )

            x0[ j ]  += tmp_x0 
            dx0[ j ] += tmp_dx0

        # If target appear!
        if t >= args.start_time + D1/2:
            is_new_goal = True 

        if is_new_goal:
            # Superimpose another submovement 
            for j in range( 3 ):
                tmp_x0, tmp_dx0, _ = min_jerk_traj( t, args.start_time + D1/2, 0, g_new[ j ] - p0f[ j ], D1 )
            
                x0[ j ]  += tmp_x0 
                dx0[ j ] += tmp_dx0        

        p0_arr.append( x0 )
        dp0_arr.append( dx0 )                

        tau  = J.T @ ( Kx @ ( x0 - xEE ) + Bx @ ( dx0 - dxEE ) )
        my_sim.mj_data.ctrl[ : ] = tau
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
    
    if args.is_save_data:
        dict  = { "t_arr": t_arr, "q_arr": q_arr, "dq_arr": dq_arr, "p_arr": p_arr, "dp_arr": dp_arr, "p0_arr": p0_arr, "dp0_arr": dp0_arr,
                    "Kx":Kx, "Bx":Bx }

        scipy.io.savemat( my_sim.tmp_dir + "/dmp.mat", { **dict } )    

    # Move the tmp folder to results if not empty, else just remove the tmp file. 
    shutil.move( my_sim.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( my_sim.tmp_dir ) ) != 0 else os.rmdir( my_sim.tmp_dir )


def run_movement_primitives( my_sim  ):

    args = my_sim.args

    # Set initial position
    q0 = np.pi/12
    init_cond = { "qpos": np.array( [ q0, np.pi - 2*q0 ] ),  "qvel": np.zeros( 2 ) }

    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    n  = my_sim.nq
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    # Define for x and y trajectory
    dmp_list = [] 

    for _ in range( 2 ):

        dmp = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )
        
        # Adding the canonical system
        dmp.add_canonical_system( cs )      

    # The parameters of min-jerk-traj
    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    p0f = p0i + np.array( [ -0.7, 0.7, 0. ] )
    D1 = 1.0



    g_old = np.copy( p0f )
    g_new = g_old + np.array( [ 1.5, 0.5, 0. ] )
    # The time constant tau is the duration of the movement. 
    cs.tau = D1 

    # The number of sample points for imitation learning
    P = 100

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = D1/P

    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 2, P + 1 ) )
    dp_des  = np.zeros( ( 2, P + 1 ) ) 
    ddp_des = np.zeros( ( 2, P + 1 ) )

    for i in range( 2 ):
        for j in range( P + 1 ):
            t = tmp_dt * j
            p_des[ i, j ], dp_des[ i, j ], ddp_des[ i, j ] = min_jerk_traj( t, 0.0, p0i[ i ], p0f[ i ], D1  )

    # The number of basis functions
    N = 20

    # Learn the weights
    for i in range( 2 ):
        t_arr = tmp_dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ], n_bfs = N )


    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    N_sim = round( args.run_time/dt  )
    p_command = np.zeros( ( 2, N_sim + 1 ) )

    for i in range( 2 ):

        dmp = dmp_list[ i ]

        y_curr = p0i[ i ]
        z_curr = 0

        for j in range( N_sim + 1 ):
            t = dt * j 

            if t <= args.start_time: 
                p_command[ i, j ] = p0i[ i ]

            else:

                # Integrate the solution 
                # Calculate the force from weights

                # Get the current canonical function value
                s = cs.get_value( t - args.start_time )

                psi_arr = np.array( [ dmp.basis_functions.calc_activation( k, s ) for k in np.arange( dmp.basis_functions.n_bfs ) ] )

                # if psi_arr is super small, then just set for as zero since this implies there is no activation
                if np.sum( psi_arr ) != 0:
                    f = np.sum( dmp.weights * psi_arr ) / np.sum( psi_arr )
                else:
                    f = 0

                # In case if f is nan, then just set f as 0 
                if math.isnan( f ): f = 0 

                f *= s * ( p0f[ i ] - p0i[ i ] ) 

                if t >= args.start_time + D1/2:
                    g = g_new + ( g_old - g_new ) * np.exp( -cs.tau * ( t - ( args.start_time + D1/2 ) ) )

                else:
                    g = g_old                    

                y_new, z_new, _ ,_ = dmp.step( g[ i ], y_curr, z_curr, f, dt )
                p_command[ i, j ] = y_new
                y_curr = y_new
                z_curr = z_new 

    # Since we now know the q_command, looping through the simulation 
    # We assume pure position control 
    t = 0.
    n_steps = 0 
    T = args.run_time 
    frames = [ ]
    
    if args.cam_pos is not None: my_sim.set_camera_pos( ) 

    t_arr = []
    q_arr = []

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
            if my_sim.mj_viewer.is_reset:
                my_sim.mj_viewer.is_reset = False
            
            # If SPACE BUTTON is pressed
            if my_sim.mj_viewer.is_paused:    continue

        px = p_command[ 0, n_steps ]
        py = p_command[ 1, n_steps ]
        
        # Solve the inverse kinematics 
        q2 = np.pi - np.arccos( 0.5 * ( 2 - px ** 2 - py ** 2  ) )
        q1 = np.arctan2( py, px ) - q2/2 

        t_arr.append( t )
        q_arr.append( np.array( [ q1, q2 ] ) )

        my_sim.mj_data.qpos[ : ] = np.array( [ q1, q2 ] )

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

    # Packing up the arrays as a dictionary
    # DMP for the first joint
    dmp1 = dmp_list[ 0 ]
    dmp2 = dmp_list[ 1 ]

    if args.is_save_data:
        dict  = { "dt": dt, "t_arr":t_arr, "p" : p_command, "q": q_arr, "tau1": dmp1.tau, "tau2": dmp2.tau, "alpha_s": cs.alpha_s,
                "weights1": dmp1.weights, "centers1": dmp1.basis_functions.centers, "heights1": dmp1.basis_functions.heights, 
                "weights2": dmp2.weights, "centers2": dmp2.basis_functions.centers, "heights2": dmp2.basis_functions.heights, 
                "alpha_z": dmp1.alpha_z, "beta_z": dmp1.beta_z }

        scipy.io.savemat( my_sim.tmp_dir + "/dmp.mat", { **dict } )                            
    
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

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 1, 0, 5, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

