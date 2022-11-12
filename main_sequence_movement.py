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

def run_motor_primitives( my_sim ):

    args = my_sim.args

    # Set initial position
    q0 = np.pi/6
    init_cond = { "qpos": np.array( [ q0, np.pi - 2 *q0 ] ),  "qvel": np.zeros( 2 ) }

    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The initial and final posture of the end effector
    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    xEEf = xEEi + np.array( [ 0., 0.5, 0. ] )

    Kx = 300 * np.eye( 3 )
    Bx = 100 * np.eye( 3 )

    # Define the raw simulation 
    T = args.run_time 

    # The movement duration of the first one
    D1 = 1

    t = 0 
    frames = [] 

    # Redefine the new time step 
    dt  = my_sim.mj_model.opt.timestep      

    is_new_goal = False

    g_new = xEEf + np.array( [ 0.5 * np.random.rand( ), 0., 0. ] )


    while t <= T + 1e-7:

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
 
        # The zero-force traejctory (3D)
        x0  = np.zeros( 3 )
        dx0 = np.zeros( 3 )

        for j in range( 3 ):
            tmp_x0, tmp_dx0, _ = min_jerk_traj( t, args.start_time, xEEi[ j ], xEEf[ j ], D1 )

            x0[ j ]  += tmp_x0 
            dx0[ j ] += tmp_dx0

        # If target appear!
        # FILL IN
        if t >= args.start_time + D1/3:
            is_new_goal = True 

        if is_new_goal:
            # Superimpose another submovement 
            for j in range( 3 ):
                tmp_x0, tmp_dx0, _ = min_jerk_traj( t, args.start_time + D1/3, 0, g_new[ j ] - xEEf[ j ], D1 )

                x0[ j ]  += tmp_x0 
                dx0[ j ] += tmp_dx0        

        tau  = J.T @ ( Kx @ ( x0 - xEE ) + Bx @ ( dx0 - dxEE ) )
        my_sim.mj_data.ctrl[ : ] = tau
        my_sim.step( )
        t += dt





def run_movement_primitives( my_sim  ):

    cs = CanonicalSystem( mov_type = "discrete" )

    # Dynamic Movement Primitives
    # First  along the x-axis
    # Second along the y-axis
    dmp1 = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )
    dmp2 = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )

    # Adding the canonical system
    # DMPs share the same canonical system 
    dmp1.add_canonical_system( cs )
    dmp2.add_canonical_system( cs )

    # Set initial position
    q0 = np.pi/6
    init_cond = { "qpos": np.array( [ q0, np.pi - 2 * q0 ] ),  "qvel": np.zeros( 2 ) }

    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The initial and final posture of the end effector
    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    xEEf = xEEi + np.array( [ 0., 0.5, 0. ] )

    g_old = np.copy( xEEf )
    g_new = xEEf + np.array( [ 0.5 * np.random.rand( ), 0., 0. ] )


    # Redefine the new time step 
    dt  = my_sim.mj_model.opt.timestep      
    t   = 0
    tau = 1
    T   = args.run_time 

    x_arr = np.zeros( round( T/dt ) + 1 )
    y_arr = np.zeros( round( T/dt ) + 1 )

    xold ,  yold = xEEi[ 0 ], xEEi[ 1 ]
    dxold, dyold =         0,         0

    idx = 0 
    print( xold, yold )

    is_new_goal = False
    D1 = 1
    
    g = g_old

    while t <= T + 1e-7:
        
        xnew, dxnew = dmp1.step( g[ 0 ], xold, dxold, 0, dt )
        ynew, dynew = dmp2.step( g[ 1 ], yold, dyold, 0, dt )

        x_arr[ idx ] = xnew
        y_arr[ idx ] = ynew

        xold, dxold = xnew, dxnew
        yold, dyold = ynew, dynew

        if t >= args.start_time + D1/3:
            g = g_new + ( g_old - g_new ) * np.exp( -1/tau * ( t - ( args.start_time + D1/3 ) ) )
        else:
            g = g_old

        idx += 1
        t += dt

    plt.plot( x_arr )
    plt.plot( y_arr )
    plt.show( )

            

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    ctrl_type = "movement"

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

