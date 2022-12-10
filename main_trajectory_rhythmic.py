"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, Joint-space trajectory planning.
|                 Section 3.2 
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import math
import shutil
import scipy.io

import numpy           as np
import moviepy.editor  as mpy

from matplotlib  import pyplot as plt
# ======================================================================== #
# ======================================================================== #
#                                                                          #
#                            ADDING LOCAL MODULES                          #
#                                                                          #
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController, CartesianImpedanceController
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

from mujoco_py    import functions as mj_functions


# ======================================================================== #
# ======================================================================== #
#
#               ADDING MODULES FOR DYNAMIC MOVEMENT PRIMITIVES             #
#
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives
from InverseDynamicsModel       import get2DOF_J, get2DOF_M, get2DOF_C, get2DOF_dJ

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       


def run_motor_primitives( my_sim ):

    # Define the joint-space impedance controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    # Get the number of actuators of the robot 
    nq = ctrl.nq

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    r = 0.5 
    omega0 = np.pi
    c = np.sqrt( 2 )
    n = my_sim.n_act
    q1 = np.arcsin( 0.5 * ( c + r ) )
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )
    ctrl.add_rhythmic_mov( amp = r, center = [ 0, c ], omega = omega0 )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):    

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "rhythmic" )

    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

    # Dynamic Movement Primitives 
    dmp_list = [] 

    # The number of basis functions
    N = 40


    # For rhythmic movement 
    # The x and y trajectory 
    r = 0.5 
    omega0 = np.pi
    c = np.sqrt( 2 )

    # The period of the system
    Tp = 2 * np.pi / omega0 
    P  = 100
    dt = Tp/P    
    t_arr = dt * np.arange( P + 1 )

    cs.tau = Tp/(2*np.pi)


    for _ in range( 2 ): 
        dmp = DynamicMovementPrimitives( mov_type = "rhythmic", cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5, tau = 1.0 )
        dmp_list.append( dmp )


    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 2, P + 1 ) )
    dp_des  = np.zeros( ( 2, P + 1 ) ) 
    ddp_des = np.zeros( ( 2, P + 1 ) )


    for i, t in enumerate( t_arr ):
        tmp_pos, tmp_vel, tmp_acc = r * np.sin( omega0 * t ), r * omega0 * np.cos( omega0 * t ), -r * ( omega0 ** 2 ) * np.sin( omega0 * t )
        p_des[   0,i ] = tmp_pos
        dp_des[  0,i ] = tmp_vel
        ddp_des[ 0,i ] = tmp_acc

        tmp_pos, tmp_vel, tmp_acc = r * np.cos( omega0 * t ) + c, -r * omega0 * np.sin( omega0 * t ), -r * ( omega0 ** 2 ) * np.cos( omega0 * t )
        p_des[   1,i ] = tmp_pos
        dp_des[  1,i ] = tmp_vel
        ddp_des[ 1,i ] = tmp_acc


    # First, learn the weights via imitation learning 
    for i in range( 2 ):
        t_arr = dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    dt = my_sim.dt    
    N_sim = round( args.run_time/dt ) + 1

    # The q, dq, ddq of the robot 
    p_command   = np.zeros( ( 2, N_sim ) )
    dp_command  = np.zeros( ( 2, N_sim ) )
    ddp_command = np.zeros( ( 2, N_sim ) )



    # Iterating through the dmps
    ttmp = [ 0 , c ]
    for i in range( 2 ):

        dmp = dmp_list[ i ]
        t_arr, y_arr, z_arr, dy_arr, dz_arr = dmp.integrate( p_des[ i, 0 ], dp_des[ i, 0 ], ttmp[ i ], dt, N_sim )

        p_command[   i, : ] =  y_arr
        dp_command[  i, : ] =  dy_arr
        ddp_command[ i, : ] =  dz_arr     

    # plt.plot( ddp_command[ 0, : ] )
    # plt.plot( ddp_command[ 1, : ] )
    # plt.show(  )

    # Since we now know the q_command, looping through the simulation 
    # We assume pure position control 
    # Defining the parameters of the simulation
    t = 0.
    T = args.run_time 

    n_steps = 0
    frames = [ ]

    px = p_command[ 0, 0 ]
    py = p_command[ 1, 0 ]

    # Solve the inverse kinematics 
    q2 = np.pi - np.arccos( 0.5 * ( 2 - px ** 2 - py ** 2  ) )
    q1 = np.arctan2( py, px ) - q2/2 

    q_arr = np.array( [ q1, q2 ] )

    # The dq_arr
    dq_arr  = np.linalg.inv( get2DOF_J( q_arr ) ) @ dp_command[ :, 0 ]

    # Forward.
    my_sim.init( qpos = q_arr, qvel = dq_arr )

    if args.cam_pos is not None: my_sim.set_camera_pos( ) 

    # Main-loop of the simulation
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

        # The q_arr 
        q_arr = np.array( [ q1, q2 ] )

        # The dq_arr
        dq_arr  = np.linalg.inv( get2DOF_J( q_arr ) ) @ dp_command[ :, n_steps ]

        # The ddq_arr
        ddq_arr = np.linalg.inv( get2DOF_J( q_arr ) ) @ ( ddp_command[ :, n_steps ] - get2DOF_dJ( q_arr, dq_arr  ) @ dq_arr )

        # Calculate the mass, coriolis matrix
        tau = get2DOF_M( q_arr  ) @ ddq_arr + get2DOF_C( q_arr, dq_arr ) @ dq_arr

        my_sim.mj_data.ctrl[ :my_sim.n_act ] = tau


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



if __name__ == "__main__":
                                                                                
    # Parse arguments given from the terminal
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    # Define the type of movement and its control method
    ctrl_type = "movement"

    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "2DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0.3, 0.6, 0, 4, -90, 90 ] )

    my_sim = Simulation( args )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

