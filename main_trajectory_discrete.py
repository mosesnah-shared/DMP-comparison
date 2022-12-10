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
from controllers  import JointImpedanceController
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
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # Get the number of actuators of the robot 
    nq = ctrl.nq

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kq = 50 * np.eye( nq ), Bq = 40 * np.eye( nq ) )

    # The parameters of min-jerk-traj
    q0i = np.zeros( nq )
    q0f =  np.ones( nq )
    D   = 2.0

    # Use minimum-jerk trajectory as the refernce trajectory 
    ctrl.add_mov_pars( q0i = q0i , q0f = q0f, D = D, ti = args.start_time  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # The initial condition of the robot and its setup
    my_sim.init( qpos = q0i, qvel = np.zeros( nq ) )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  ctrl.export_data( my_sim.tmp_dir )

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

    # The number of basis functions
    N = 10

    for _ in range( nq ): 
        dmp = DynamicMovementPrimitives( mov_type = "discrete", cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5, tau = 1.0 )
        dmp_list.append( dmp )

    # The parameters of min-jerk-traj
    q0i = np.zeros( nq )
    q0f =  np.ones( nq )
    D   = 2.0

    # The time constant tau is the duration of the movement. 
    cs.tau = D        

    # The number of sample points for imitation learning
    P = 100

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = D/P

    # The P samples points of q_des, dq_des, ddq_des
    q_des   = np.zeros( ( nq, P + 1 ) )
    dq_des  = np.zeros( ( nq, P + 1 ) ) 
    ddq_des = np.zeros( ( nq, P + 1 ) )

    for i in range( nq ):
        for j in range( P + 1 ):
            t = tmp_dt * j
            q_des[ i, j ], dq_des[ i, j ], ddq_des[ i, j ] = min_jerk_traj( t, 0.0, q0i[ i ], q0f[ i ], D  )

    # First, learn the weights via imitation learning 
    for i in range( nq ):
        t_arr = tmp_dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, q_des[ i, : ], dq_des[ i, : ], ddq_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    N_sim = round( args.run_time/dt ) + 1

    # The q, dq, ddq of the robot 
    q_command   = np.zeros( ( nq, N_sim ) )
    dq_command  = np.zeros( ( nq, N_sim ) )
    ddq_command = np.zeros( ( nq, N_sim ) )

    # Iterating through the dmps
    for i in range( nq ):

        dmp = dmp_list[ i ]

        # y, z, dy, dz
        t_arr, y_arr, z_arr, dy_arr, dz_arr = dmp.integrate( q0i[ i ], 0, q0f[ i ], dt, N_sim )

        q_command[   i, : ] =  y_arr
        dq_command[  i, : ] =  dy_arr
        ddq_command[ i, : ] =  dz_arr 

    # Since we now know the q_command, looping through the simulation 
    # We assume pure position control 
    # Defining the parameters of the simulation
    t = 0.
    T = args.run_time 

    n_steps = 0
    frames = [ ]
    my_sim.init( qpos = q0i, qvel = np.zeros( nq ) )
    
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

        # Calculate the mass, coriolis matrix
        tau = get2DOF_M( q_command[ :, n_steps ]  ) @ ddq_command[ :, n_steps ] + \
              get2DOF_C( q_command[ :, n_steps ], dq_command[ :, n_steps ] ) @ dq_command[ :, n_steps ]

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

    

