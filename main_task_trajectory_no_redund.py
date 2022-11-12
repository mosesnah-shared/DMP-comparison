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
import shutil
import numpy      as np
import scipy.io
from datetime  import datetime
from matplotlib  import pyplot as plt

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController
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

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

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
        ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 2, ti = args.start_time  )    

    else:
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


def run_movement_primitives( mov_type ):

    cs = CanonicalSystem( mov_type = mov_type )

    # Dynamic Movement Primitives
    # First  along the x-axis
    # Second along the y-axis
    dmp1 = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )
    dmp2 = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )

    # Adding the canonical system
    # DMPs share the same canonical system 
    dmp1.add_canonical_system( cs )
    dmp2.add_canonical_system( cs )

    if mov_type == "discrete":

        # The initial posture is at ( q1, q2 ) = ( pi/4, pi/2 )
        # Hence, the initial XY position: x_i = 0, y_i = sqrt( 2 ) 

        # The final XY position depends on the length of the movement 
        # We set this as lm 
        idx = 0         # 0~7 for targets 
        l   = 0.5

        idx = args.target_idx

        xEEi = np.array( [ 0.0, np.sqrt( 2 ) ] )
        xEEf = xEEi + l * np.array( [ np.cos( idx * np.pi/4 ), np.sin( idx * np.pi/4 ) ] )

        # Duration of the movement is D
        D = 2
        
        # The time step for imitation learning is 0.01. 
        dt = 0.01

        # Offset of time 0.3 was added
        T  = D + 0.3
        N = round( T/dt )
        t_arr = dt * np.arange( N )        

        # The first DMP is for the x direction
        x_des   = np.zeros( N )
        dx_des  = np.zeros( N )
        ddx_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = min_jerk_traj( t, 0.0, xEEi[ 0 ], xEEf[ 0 ], D  )
            x_des[   i ] = tmp_pos
            dx_des[  i ] = tmp_vel
            ddx_des[ i ] = tmp_acc

        # The second DMP is for the y direction
        y_des   = np.zeros( N )
        dy_des  = np.zeros( N )
        ddy_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = min_jerk_traj( t, 0.0, xEEi[ 1 ], xEEf[ 1 ], D  )
            y_des[   i ] = tmp_pos
            dy_des[  i ] = tmp_vel
            ddy_des[ i ] = tmp_acc            

        # Setting up the canonical system's parameters
        cs.tau = D

        n_bfs = 100
        dmp1.imitation_learning( t_arr, x_des, dx_des, ddx_des, n_bfs = n_bfs )
        dmp2.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )

        t_arr1, y_arr1, z_arr1, dy_arr1, dz_arr1 = dmp1.integrate( xEEi[ 0 ], 0, xEEf[ 0 ], 0.001, round( T/0.001 ) )   
        t_arr2, y_arr2, z_arr2, dy_arr2, dz_arr2 = dmp2.integrate( xEEi[ 1 ], 0, xEEf[ 1 ], 0.001, round( T/0.001 ) )   



    else:
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
        n_bfs = 40
        dmp1.imitation_learning( t_arr, x_des, dx_des, ddx_des, n_bfs = n_bfs )
        dmp2.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )

        t_arr1, y_arr1, z_arr1, dy_arr1, dz_arr1 = dmp1.integrate( x_des[ 0 ], dx_des[ 0 ], 0, 0.001, round( 2 * Tp/0.001 ) )   
        t_arr2, y_arr2, z_arr2, dy_arr2, dz_arr2 = dmp2.integrate( y_des[ 0 ], dy_des[ 0 ], c, 0.001, round( 2 * Tp/0.001 ) )   




    # Stuffs for saving the data
    if args.is_save_data:
        tmp_dir  = C.TMP_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S/" )
        os.mkdir( tmp_dir )  

        file_name = tmp_dir + "/dmp1.mat"
        
        # Packing up the arrays as a dictionary
        # DMP for the first movement
        dict1  = { "mov_type" : mov_type, "t_des1": t_arr, "y_des1": x_des, "dy_des1": dx_des, "ddy_des1": ddx_des, "n_bfs1": n_bfs, 
                "t_arr1": t_arr1, "y_arr1": y_arr1, "z_arr1": z_arr1, "dy_arr1": dy_arr1, "dz_arr1": dz_arr1, "tau1": dmp1.tau, 
                "weights1": dmp1.weights, "centers1": dmp1.basis_functions.centers, "heights1": dmp1.basis_functions.heights, 
                 "alpha_z1": dmp1.alpha_z, "beta_z1": dmp1.beta_z }

        dict2  = { "mov_type" : mov_type, "t_des2": t_arr, "y_des2": y_des, "dy_des2": dy_des, "ddy_des2": ddy_des, "n_bfs2": n_bfs, 
                "t_arr2": t_arr2, "y_arr2": y_arr2, "z_arr2": z_arr2, "dy_arr": dy_arr2, "dz_arr2": dz_arr2, "tau2": dmp2.tau, 
                "weights2": dmp2.weights, "centers2": dmp2.basis_functions.centers, "heights2": dmp2.basis_functions.heights, 
                 "alpha_z2": dmp2.alpha_z, "beta_z2": dmp2.beta_z }                 

        scipy.io.savemat( file_name, { **dict1, **dict2 } )    

        # Move the tmp folder to results if not empty, else just remove the tmp file. 
        shutil.move( tmp_dir, C.SAVE_DIR  ) if len( os.listdir( tmp_dir ) ) != 0 else os.rmdir( tmp_dir )
            

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    mov_type = "rhythmic"
    ctrl_type = "movement"

    if    ctrl_type == "motor"   :    run_motor_primitives( mov_type )
    elif  ctrl_type == "movement": run_movement_primitives( mov_type )

    

