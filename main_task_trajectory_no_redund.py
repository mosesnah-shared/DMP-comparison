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

def run_motor_primitives( my_sim, mov_type ):


    # Define the controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "task_imp" )
    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    if mov_type == "discrete":

        n = my_sim.n_act
        q1 = np.pi * 1/4
        init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( n ) }
        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

        xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
        idx = args.target_idx
        xEEf = xEEi + 0.5 * np.array( [ np.cos( idx * np.pi/4 ), np.sin( idx * np.pi/4 ), 0 ] )
        ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 1, ti = args.start_time  )    

    elif mov_type == "rhythmic":
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

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim, mov_type ):


    # Define the canonical system
    cs = CanonicalSystem( mov_type = mov_type )

    n  = my_sim.nq
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    # Define for x and y trajectory
    dmp_list = [] 
    for _ in range( 2 ):
        dmp = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )
        
        # Adding the canonical system
        dmp.add_canonical_system( cs )      

    if mov_type == "discrete":

        q1 = np.pi * 1/4
        init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( n ) }
        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

        # The parameters of min-jerk-traj
        p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
        idx = args.target_idx
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

        # The number of basis functions
        N = 10

        # The time array for imitation learning
        # This learns the best fit weight of the dmp
        # For this, we need to define the 

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

                    y_new, z_new, _ ,_ = dmp.step( p0f[ i ], y_curr, z_curr, f, dt )
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
        
        # Saving the data for analysis
        if args.is_save_data:
            
            # Packing up the arrays as a dictionary
            # DMP for the first joint
            dmp1 = dmp_list[ 0 ]
            dmp2 = dmp_list[ 1 ]

            dict  = { "mov_type" : mov_type, "dt": dt, "x" : p_command[ 0, : ], "y" : p_command[ 1, : ], "tau1": dmp1.tau, "tau2": dmp2.tau, "alpha_s": cs.alpha_s,
                    "weights1": dmp1.weights, "centers1": dmp1.basis_functions.centers, "heights1": dmp1.basis_functions.heights, 
                    "weights2": dmp2.weights, "centers2": dmp2.basis_functions.centers, "heights2": dmp2.basis_functions.heights, 
                    "alpha_z": dmp1.alpha_z, "beta_z": dmp1.beta_z }

            scipy.io.savemat( my_sim.tmp_dir + "/dmp.mat", { **dict } )    

        # Move the tmp folder to results if not empty, else just remove the tmp file. 
        shutil.move( my_sim.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( my_sim.tmp_dir ) ) != 0 else os.rmdir( my_sim.tmp_dir )


    elif mov_type == "rhythmic":
        # For rhythmic movement 
        # The x and y trajectory 
        r = 0.5 
        omega0 = np.pi
        c = np.sqrt( 2 )

        # The period of the system
        Tp = 2 * np.pi / omega0 
        dt = 0.01
        N  = round( Tp/dt )
        t_arr = dt * np.arange( N )

        # The first DMP is for the x direction
        x_des   = np.zeros( N )
        dx_des  = np.zeros( N )
        ddx_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = r * np.sin( omega0 * t ), r * omega0 * np.cos( omega0 * t ), -r * ( omega0 ** 2 ) * np.sin( omega0 * t )
            x_des[   i ] = tmp_pos
            dx_des[  i ] = tmp_vel
            ddx_des[ i ] = tmp_acc

        # The second DMP is for the x direction
        y_des   = np.zeros( N )
        dy_des  = np.zeros( N )
        ddy_des = np.zeros( N )            

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = r * np.cos( omega0 * t ) + c, -r * omega0 * np.sin( omega0 * t ), -r * ( omega0 ** 2 ) * np.cos( omega0 * t )
            y_des[   i ] = tmp_pos
            dy_des[  i ] = tmp_vel
            ddy_des[ i ] = tmp_acc

        # Setting up the canonical system's parameters
        cs.tau = Tp / ( 2 * np.pi)

        # The number of Basis Functions
        # n_bfs = 40
        # dmp1.imitation_learning( t_arr, x_des, dx_des, ddx_des, n_bfs = n_bfs )
        # dmp2.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )

        # t_arr1, y_arr1, z_arr1, dy_arr1, dz_arr1 = dmp1.integrate( x_des[ 0 ], dx_des[ 0 ], 0, 0.001, round( 2 * Tp/0.001 ) )   
        # t_arr2, y_arr2, z_arr2, dy_arr2, dz_arr2 = dmp2.integrate( y_des[ 0 ], dy_des[ 0 ], c, 0.001, round( 2 * Tp/0.001 ) )   

            

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    mov_type = "discrete"
    ctrl_type = "motor"
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

    assert mov_type  in [ "discrete", "rhythmic" ]
    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "2DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 1, 0, 5, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim, mov_type )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim, mov_type )

    

