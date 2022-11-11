"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, Joint-space trajectory planning.
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import shutil
import numpy      as np
import scipy.io
from datetime  import datetime
from matplotlib  import pyplot as plt

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives


# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )


def run_motor_primitives( mov_type ):

    args.model_name = "1DOF_planar_torque"

    my_sim = Simulation( args )

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kq = np.array( [ 50.0 ] ), Bq = np.array( [ 25.0 ] ) )

    if mov_type == "discrete":
        # Minimum Jerk Trajectory
        ctrl.add_mov_pars( q0i = np.array( [ 0. ] ) , q0f = np.array( [ 1.0 ] ), D = 1, ti = args.start_time  )    

    else:
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
    init_cond = { "qpos": np.array( [ 0 ] ) ,  "qvel": np.array( [ 1 ] ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( mov_type ):    

    # Define the canonical system
    cs = CanonicalSystem( mov_type = mov_type )

    # Dynamic Movement Primitives
    dmp = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )

    # Adding the canonical system
    dmp.add_canonical_system( cs )

    if mov_type == "discrete":

        # Train the movement as minimum jerk trajectory 
        # The total time T = N * 0.01
        N  = 10
        dt = 0.01
        T  = N * dt
        t_arr = dt * np.arange( N )
        pi, pf, D = 0.0, 1.0, 1.0

        # def min_jerk_traj( t: float, ti: float, tf: float, pi: float, pf: float, D: float ):
        y_des   = np.zeros( N )
        dy_des  = np.zeros( N )
        ddy_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = min_jerk_traj( t, 0.0, D, pi, pf, D  )
            y_des[   i ] = tmp_pos
            dy_des[  i ] = tmp_vel
            ddy_des[ i ] = tmp_acc

        # Setting up the canonical system's parameters
        cs.tau = D

        n_bfs = 10
        dmp.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )


        t_arr2, y_arr, z_arr, dy_arr, dz_arr = dmp.integrate( 0, 0, pf, 0.001, int( T/0.001 ) )   

        plt.plot( t_arr2, y_arr, linestyle="dashed" )         
        # plt.plot( t_arr2, dz_arr, linestyle="dashed" )         
        # plt.plot( t_arr2, z_arr )         

        plt.plot(  t_arr, y_des )         
        # plt.plot( t_arr, dy_des )         
        # plt.plot( t_arr, ddy_des )                 

        plt.show( )
    else:

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

    # Stuffs for saving the data
    if args.is_save_data:
        tmp_dir  = C.TMP_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S/" )
        os.mkdir( tmp_dir )  

        file_name = tmp_dir + "/dmp1.mat"
        
        # Packing up the arrays as a dictionary
        # DMP for the first joint
        dict  = { "mov_type" : mov_type, "t_des": t_arr, "y_des": y_des, "dy_des": dy_des, "ddy_des": ddy_des, "n_bfs": n_bfs, 
                "t_arr": t_arr2, "y_arr": y_arr, "z_arr": z_arr, "dy_arr": dy_arr, "dz_arr": dz_arr, "tau": dmp.tau, 
                "weights": dmp.weights, "centers": dmp.basis_functions.centers, "heights": dmp.basis_functions.heights, 
                 "alpha_z": dmp.alpha_z, "beta_z": dmp.beta_z }
        
        scipy.io.savemat( file_name, { **dict } )    

        # Move the tmp folder to results if not empty, else just remove the tmp file. 
        shutil.move( tmp_dir, C.SAVE_DIR  ) if len( os.listdir( tmp_dir ) ) != 0 else os.rmdir( tmp_dir )
            

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    mov_type  = "rhythmic"
    ctrl_type = "motor"

    if    ctrl_type == "motor"   :    run_motor_primitives( mov_type )
    elif  ctrl_type == "movement": run_movement_primitives( mov_type )

    

