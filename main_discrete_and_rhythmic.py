"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, the most basic script
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
from controllers  import JointImpedanceController 
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

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    ctrl.set_impedance( Kq = np.diag( [ 300, 100 ] ), Bq = np.diag( [ 300, 30 ] ) )
    n = my_sim.n_act

    mov_arrs  = np.array(  [  0.6, 0.5, 0.6, 1.5, 1. ] )     
    toff = 3.5

    ctrl.add_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = 1 * toff  )    
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ :n ] - mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = 2 * toff )        
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ n:2*n ] - mov_arrs[ :n ], D = mov_arrs[ -1 ], ti = 3 * toff )        
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ :n ] - mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = 4 * toff )        
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ n:2*n ] - mov_arrs[ :n ], D = mov_arrs[ -1 ], ti = 5 * toff )        
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ :n ] - mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = 6 * toff )           
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ n:2*n ] - mov_arrs[ :n ], D = mov_arrs[ -1 ], ti = 7 * toff )        

    # Add rhythmic movements too
    ctrl.add_rhythmic_mov( amp = np.array( [ 0., 0.3 ] ), offset = np.array( [ 0., 0. ] ), omega = 2 * np.pi  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    init_cond = { "qpos": mov_arrs[ :n ] ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):

    # Generate DMP for rhythmic and goal-directed 
    args = my_sim.args

    # Set initial position
    mov_arrs  = np.array(  [  0.6, 0.5, 0.6, 1.5, 1. ] )     
    n = my_sim.n_act
    init_cond = { "qpos": mov_arrs[ :n ],  "qvel": np.zeros( 2 ) }

    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Define the canonical system
    cs   = CanonicalSystem( mov_type = "rhythmic" )
    cs_g = CanonicalSystem( mov_type = "discrete" )

    dmp   = DynamicMovementPrimitives( mov_type = "rhythmic", alpha_z = 10, beta_z = 2.5 )
    dmp_g = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )

    dmp.add_canonical_system( cs )      
    dmp_g.add_canonical_system( cs_g )      

    # The number of sample points for imitation learning
    P = 100

    omega = 2 * np.pi
    Tp = 2 * np.pi / omega

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = Tp/P

    cs.tau = 1/ omega    

    # The P samples points of p_des, dp_des, ddp_dex
    q_des   = np.zeros( P + 1 )
    dq_des  = np.zeros( P + 1 ) 
    ddq_des = np.zeros( P + 1 )
    amp = 0.3

    for i in range( P + 1 ):
        t = tmp_dt * i
        q_des[   i ] =  mov_arrs[ 1 ] + amp * np.sin( omega * t )             
        dq_des[  i ] =  amp * np.cos( omega * t ) * omega     
        ddq_des[ i ] = -amp * np.sin( omega * t ) * omega ** 2

        # The number of basis functions
        N = 50

        # The time array for imitation learning
        # This learns the best fit weight of the dmp
        # For this, we need to define the 


    t_arr = tmp_dt * np.arange( P + 1 )
    dmp.imitation_learning( t_arr, q_des[ : ], dq_des[ : ], ddq_des[ : ], n_bfs = N )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    dt = my_sim.dt
    N_sim = round( args.run_time/dt  )

    q_command   = np.zeros( N_sim + 1 )
    dq_command  = np.zeros( N_sim + 1 )
    ddq_command = np.zeros( N_sim + 1 )

    y_curr = q_des[ 0 ]
    z_curr = dq_des[ 0 ]

    g0 = mov_arrs[ 1 ]

    g_curr  = g0
    dg_curr = 0 

    n_cnt = 0

    t_off = 3.5
    n_cnt = 1

    g0_arr = []
    g_arr  = []

    for i in range( N_sim + 1 ):
        t = dt * i

        # Get the current canonical function value
        s = cs.get_value( t )

        psi_arr = np.array( [ dmp.basis_functions.calc_activation( k, s ) for k in np.arange( dmp.basis_functions.n_bfs ) ] )

        # if psi_arr is super small, then just set for as zero since this implies there is no activation
        if np.sum( psi_arr ) != 0:
            f = np.sum( dmp.weights * psi_arr ) / np.sum( psi_arr )
        else:
            f = 0

        # In case if f is nan, then just set f as 0 
        if math.isnan( f ): f = 0 

        if t >= t_off * n_cnt:
            if   np.mod( n_cnt, 2 ) == 1:
                g0 += 1.0
            elif np.mod( n_cnt, 2 ) == 0:
                g0 -= 1.0
            n_cnt += 1 

        g0_arr.append( g0 )
        g_arr.append( g_curr )

        g_new, dg_new, dg, ddg = dmp_g.step( g0, g_curr, dg_curr, 0, dt )
        g_curr = g_new 
        dg_curr = dg_new 

        y_new, z_new, dy, dz = dmp.step( g_new, y_curr, z_curr, f, dt )


        q_command[ i ]   = y_new
        dq_command[ i ]  = z_new / cs.tau
        ddq_command[ i ] = dz / cs.tau

        y_curr = y_new
        z_curr = z_new     

    # Since we now know the q_command, looping through the simulation 
    # We assume pure position control 
    t       = 0.
    n_steps = 0 
    T       = args.run_time 
    frames  = [ ]
    
    if args.cam_pos is not None: my_sim.set_camera_pos( ) 

    t_arr  = []
    q_arr  = []


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

        my_sim.mj_data.qpos[ : ] = np.array( [ mov_arrs[ 0 ], q_command[ n_steps ] ] )
        q_arr.append( np.copy(  my_sim.mj_data.qpos[ : ] )  )
        t_arr.append( t )

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

    if args.is_save_data:
        dict  = { "dt": dt, "t_arr":t_arr, "q": q_arr, "g0": g0_arr , "g": g_arr } 

        scipy.io.savemat( my_sim.tmp_dir + "/dmp.mat", { **dict } )                            
    
    # Move the tmp folder to results if not empty, else just remove the tmp file. 
    shutil.move( my_sim.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( my_sim.tmp_dir ) ) != 0 else os.rmdir( my_sim.tmp_dir )


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

