"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Obstacle Avoidance
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
from controllers  import CartesianImpedanceController, CartesianImpedanceControllerObstacle
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

def run_movement_primitives( my_sim ):

    args = my_sim.args

    # Set initial position
    q0 = np.pi/12
    init_cond = { "qpos": np.array( [ q0, np.pi - 2*q0 - 0.02 ] ),  "qvel": np.zeros( 2 ) }

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
    p0f = p0i + np.array( [ 0., 1.2, 0. ] )

    # Obstacle Location
    o = np.array( [ 0, 0.5 * (p0i[ 1 ] + p0f[ 1 ]), 0 ] )

    g   = np.copy( p0f )
    D1 = 3.0

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
    p_command  = np.zeros( ( 2, N_sim + 1 ) )
    dp_command = np.zeros( ( 2, N_sim + 1 ) )

    tmp_p  = np.array( [ p0i[ 0 ], p0i[ 1 ], 0 ] )
    tmp_dp = np.zeros( 3 )

    y_curr = tmp_p 
    z_curr = tmp_dp 

    for i in range( N_sim + 1 ):

        # Calculate the coupling term 
        # Need to know the current p and dp 
        if np.sum( tmp_dp ) != 0:
            theta = np.arccos( np.inner( o - tmp_p ,tmp_dp  ) / ( np.linalg.norm( o - tmp_p ) * np.linalg.norm( tmp_dp)  )   )

            R = np.array( [ [ 0, 1, 0 ], [ -1, 0, 0 ], [0 ,0, 1 ] ] )
            
            Cp = 300 * R @ tmp_dp * np.exp( -3 * theta )

        else:
            Cp = np.zeros( 3 )

        for j in range( 2 ):

            dmp = dmp_list[ j ]
            t = dt * i

            if t <= args.start_time: 
                p_command[ j, i ] = p0i[ j ]

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

                f *= s * ( p0f[ j ] - p0i[ j ] ) 
                f += Cp[ j ]

                y_new, z_new, _ ,_ = dmp.step( g[ j ], y_curr[ j ], z_curr[ j ], f, dt )
                p_command[ j, i ]  = y_new
                dp_command[ j, i ] = z_new / cs.tau

                y_curr[ j ] = y_new
                z_curr[ j ] = z_new 
            

        tmp_p  = np.zeros( 3 )
        tmp_dp = np.zeros( 3 )

        tmp_p[ 0:2 ]  = np.copy( p_command[ :, i ] )
        tmp_dp[ 0:2 ] = np.copy( dp_command[ :, i ] )

    # Since we now know the q_command, looping through the simulation 
    # We assume pure position control 
    t = 0.
    n_steps = 0 
    T = args.run_time 
    frames = [ ]

    t_arr = []
    q_arr = []
    
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


def run_motor_primitives( my_sim  ):

    # Add the movements of the Cartesian Impedance Controller 
    # Set the initial posture 
    n = my_sim.n_act
    q1 = np.pi * 1/12
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1- 0.02] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 

    # The end-effector position for the final position
    p0f = p0i + np.array( [ 0.0, 1.2, 0.0 ] )

    # Define a Task-space controller 1
    ctrl  = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    o = np.array( [ 0, 0.5 * (p0i[ 1 ] + p0f[ 1 ]), 0 ] )

    # Define an impedance controller for obstacle avoidance
    ctrl2 = CartesianImpedanceControllerObstacle( my_sim, args, name = "task_imp2", obs_pos = o   )
    ctrl2.set_impedance( k = 0.1 )
    ctrl2.set_order( n = 6 )

    ctrl.add_mov_pars( x0i = p0i, x0f = p0f, D = 3.0, ti = args.start_time  )    
    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.add_ctrl( ctrl2 )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    ctrl_type = "motor"
                                                                                
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

    

