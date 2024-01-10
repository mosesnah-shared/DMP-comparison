"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Controlling Position and Orientation
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import numpy      as np
import mujoco_py  as mjPy

# The Local Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )
from utils       import min_jerk_traj
from constants   import Constants as C
from constants   import my_parser
from simulation  import Simulation

# Modules for DMP
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )
from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( args ):

    # The model name
    model_name =  "iiwa14/iiwa14"

    # Construct the basic mujoco attributes
    mj_model  = mjPy.load_model_from_path( C.MODEL_DIR + model_name + ".xml" ) 
    mj_sim    = mjPy.MjSim( mj_model )    
    mj_data   = mj_sim.data                                              
    mj_viewer = mjPy.MjViewerBasic( mj_sim ) if not args.is_vid_off else None  
    
    # The basic info of the model
    n_act    = len( mj_model.actuator_names )
    nq       = len( mj_model.joint_names    )
    dt       = mj_model.opt.timestep                               
    T        = args.run_time    
    fps      = 60   
    n_steps  = 0
    vid_step = round( ( 1. / dt ) / ( fps / args.vid_speed )  )
    
    
    # Setting the initial position and velocity of the robot 
    qpos = np.array( [ 0 , np.pi/6 , 0 , -np.pi/3 , 0, np.pi/2, 0 ] )
    qvel = np.zeros( nq )

    # If the array is shorter than the actual self.nq in the model, just fill it with zero 
    if n_act != 1:
        mj_data.qpos[ : ] = qpos[ : nq ] if len( qpos ) >= nq else np.concatenate( ( qpos, np.zeros( nq - len( qpos ) ) ) , axis = None )
        mj_data.qvel[ : ] = qvel[ : nq ] if len( qvel ) >= nq else np.concatenate( ( qvel, np.zeros( nq - len( qvel ) ) ) , axis = None )
    else:
        mj_data.qpos[ 0 ] = qpos
        mj_data.qvel[ 0 ] = qvel

    # Forward the simulation to update the posture 
    mj_sim.forward( )

    # The end-effector name
    EE_name = "iiwa14_right_hand"
    
    # Get the initial end-effector position and orientation
    p_init = np.copy( mj_data.get_body_xpos( EE_name ) )
    R_init = np.copy( mj_data.get_body_xmat( EE_name ) )

    # The main loop of the simulation 
    while mj_data.time <= T + 1e-7:

        # Render the simulation if mj_viewer exists        
        if mj_viewer is not None and n_steps % vid_step == 0:
            mj_viewer.render( )
            print( mj_data.time )

        # Joint position and velocity
        q_curr  = np.copy( mj_data.qpos[ : n_act ] )
        dq_curr = np.copy( mj_data.qvel[ : n_act ] )

        # End-effector position and velocity, translational
        p_curr  = np.copy( mj_data.get_body_xpos(  EE_name ) )
        dp_curr = np.copy( mj_data.get_body_xvelp( EE_name ) )
        Jp      = np.copy( mj_data.get_body_jacp(  EE_name ).reshape( 3, -1 ) )

        # The end-effector virtual translational trajectory
        # For this example, we use the minimum-jerk trajectory 

        # End-effector position and velocity, rotational


        # Add the Controller here
        # Controller 1: Joint-space Damping
        tau1 = -5 * dq_curr
        tau2 = Jp.T @ ( 400 * ( p_init - p_curr ) + 40 ** ( - dp_curr ) )
        # tau3 = 
        # tau = tau1 + tau2 + tau3
        tau = tau1 + tau2
        mj_data.ctrl[ :n_act ] = tau

        # Run a single simulation 
        mj_sim.step( )
        n_steps += 1




def run_movement_primitives( my_sim ):
    NotImplementedError( )

if __name__ == "__main__":
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    assert args.sim_type in [  "motor", "movement" ]

    # Set the camera position of the simulation
    if    args.sim_type == "motor"   :    run_motor_primitives( args )
    elif  args.sim_type == "movement": run_movement_primitives( args )


