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
import numpy      as np
import scipy.io
from datetime  import datetime
from matplotlib  import pyplot as plt
import moviepy.editor  as mpy

# ======================================================================== #
# ======================================================================== #
#
#                            ADDING LOCAL MODULES                          #
#
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
#
#               ADDING MODULES FOR DYNAMIC MOVEMENT PRIMITIVES             #
#
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( my_sim, mov_type ):

    # Define the joint-space impedance controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # Get the number of actuators of the robot 
    nq = ctrl.nq

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kq = 50 * np.eye( nq ), Bq = 40 * np.eye( nq ) )

    if   mov_type == "discrete":

        # The parameters of min-jerk-traj
        q0i = np.zeros( nq )
        q0f =  np.ones( nq )
        D   = 2.0

        # Use minimum-jerk trajectory as 
        ctrl.add_mov_pars( q0i = q0i , q0f = q0f, D = D, ti = args.start_time  )    

    elif mov_type == "rhythmic":
        # Multiple rhythmic movements
        # np.sin( 2 pi * t ) 
        ctrl.add_rhythmic_mov( amp = np.array( [ 1.0 ] ), offset = np.array( [ 0.0 ] ), omega = 1. ) #2 * np.pi  )    

        # 0.25 * np.sin( 4pi t + 0.77 + np.pi/2)                      
        # ctrl.add_rhythmic_mov( amp = np.array( [ -0.25 ] ), offset = np.array( [ 0.77 - np.pi/2 ] ), omega = 4 * np.pi  )    

        # 0.1 * np.sin( 6pi t + 3.0 )                      
        # ctrl.add_rhythmic_mov( amp = np.array( [ 0.10 ] ), offset = np.array( [ 3.0 ] ), omega = 6 * np.pi  )    


    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # The initial condition of the robot and its setup
    my_sim.init( qpos = q0i, qvel = np.zeros( nq ) )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim, mov_type ):    

    # Define the canonical system
    cs = CanonicalSystem( mov_type = mov_type )

    nq = my_sim.nq
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    dmp_list = [] 
    for _ in range( nq ):
        dmp = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )
        
        # Adding the canonical system
        dmp.add_canonical_system( cs )

    if mov_type == "discrete":

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

    
        # The number of basis functions
        N = 10

        # The time array for imitation learning
        # This learns the best fit weight of the dmp
        # For this, we need to define the 

        for i in range( nq ):
            t_arr = tmp_dt * np.arange( P + 1 )
            dmp = dmp_list[ i ]
            dmp.imitation_learning( t_arr, q_des[ i, : ], dq_des[ i, : ], ddq_des[ i, : ], n_bfs = N )

        # Now, we integrate this solution
        # For this, the initial and final time of the simulation is important
        N_sim = round( args.run_time/dt  )
        q_command = np.zeros( ( nq, N_sim + 1 ) )

        for i in range( nq ):

            dmp = dmp_list[ i ]

            y_curr = q0i[ i ]
            z_curr = 0

            for j in range( N_sim + 1 ):
                t = dt * j 

                if t <= args.start_time: 
                    q_command[ i, j ] = q0i[ i ]

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

                    f *= s * ( q0f[ i ] - q0i[ i ] ) 

                    y_new, z_new ,_ ,_ = dmp.step( q0f[ i ], y_curr, z_curr, f, dt )
                    q_command[ i, j ] = y_new
                    y_curr = y_new
                    z_curr = z_new 

        # Since we now know the q_command, looping through the simulation 
        # We assume pure position control 
        t = 0.
        n_steps = 0
        T = args.run_time 
        frames = [ ]
        my_sim.init( qpos = q0i, qvel = np.zeros( nq ) )
        
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

            my_sim.mj_data.qpos[ : ] = q_command[ :, n_steps ]

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

    elif mov_type == "rhythmic":


        # Train the movement as minimum jerk trajectory 
        # The total time T = N * 0.01

        omega = 1 # np.pi

        # The period (Tp) of the rhythmic movement is defined
        Tp = 2. * np.pi / omega

        # The Total time T
        T  = 3. * Tp
        dt = 0.01
        N  = int( Tp/dt )
        t_arr = dt * np.arange( N )
        y_des   = np.zeros( N )
        dy_des  = np.zeros( N )
        ddy_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            y_des[   i ] =  np.sin( omega * t )             # - 0.25 * np.sin( 2 * omega * t + 0.77 - np.pi/2 )                      + 0.1 * np.sin( 3 * omega * t + 3.0)
            dy_des[  i ] =  np.cos( omega * t ) * omega     # - 0.25 * ( 2 * omega ) * np.cos( 2 * omega * t + 0.77 - np.pi/2 )      + 0.1 * 3 * omega * np.cos( 3 * omega * t + 3.0)
            ddy_des[ i ] = -np.sin( omega * t ) * omega ** 2# + 0.25 * ( 2 * omega ) ** 2 * np.sin( 2 * omega * t + 0.77 - np.pi/2 ) - 0.1 * ( 3 * omega ) ** 2 * np.sin( 3 * omega * t + 3.0)

        # Setting up the canonical system's parameters
        cs.tau = Tp / ( 2 * np.pi) 

        # The number of Basis Functions
        n_bfs = 15
        dmp.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )

        t_arr2, y_arr, z_arr, dy_arr, dz_arr = dmp.integrate( y_des[ 0 ], dy_des[ 0 ], 0.5 * np.min( y_des ) + 0.5 * np.max( y_des ), 0.001, round( T / 0.001 ) )                    
        

        plt.plot( t_arr2, y_arr, linestyle="dashed" )         
        # plt.plot( t_arr2, dz_arr, linestyle="dashed" )         
        # plt.plot( t_arr2, z_arr )         

        plt.plot(  t_arr, y_des )         
        # plt.plot( t_arr, dy_des )         
        # plt.plot( t_arr, ddy_des )                 

        plt.show( )

    # Saving the data for analysis
    if args.is_save_data:
        
        # Packing up the arrays as a dictionary
        # DMP for the first joint
        dmp1 = dmp_list[ 0 ]
        dmp2 = dmp_list[ 0 ]

        dict1  = { "mov_type" : mov_type, "dt": dt, "q_result1" : q_command[ 0, : ], "tau": dmp1.tau, "alpha_s": cs.alpha_s,
                "weights": dmp1.weights, "centers": dmp1.basis_functions.centers, "heights": dmp1.basis_functions.heights, 
                 "alpha_z": dmp1.alpha_z, "beta_z": dmp1.beta_z }

        dict2  = { "mov_type" : mov_type,"dt": dt, "q_result2" : q_command[ 1, : ], "tau": dmp2.tau, "alpha_s": cs.alpha_s,
                        "weights": dmp2.weights, "centers": dmp2.basis_functions.centers, "heights": dmp2.basis_functions.heights, 
                        "alpha_z": dmp2.alpha_z, "beta_z": dmp2.beta_z }                 

        
        scipy.io.savemat( my_sim.tmp_dir + "/dmp1.mat", { **dict1 } )    
        scipy.io.savemat( my_sim.tmp_dir + "/dmp2.mat", { **dict2 } )    

        # Move the tmp folder to results if not empty, else just remove the tmp file. 
        shutil.move( my_sim.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( my_sim.tmp_dir ) ) != 0 else os.rmdir( my_sim.tmp_dir )
            

if __name__ == "__main__":
                                                                                
    # Parse arguments given from the terminal
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    # Define the type of movement and its control method
    mov_type  = "discrete"
    ctrl_type = "motor"

    assert mov_type  in [ "discrete", "rhythmic" ]
    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "2DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0.3, 0.6, 0, 4, -90, 90 ] )

    my_sim = Simulation( args )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim, mov_type )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim, mov_type )

    

