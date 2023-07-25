import os
import sys
import scipy.io
import numpy    as np
from mujoco_py import functions

from   modules.utils     import min_jerk_traj

# The modules Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "../DMPmodules" ) )
sys.path.append( os.path.join( os.path.dirname(__file__), "../inversemodel" ) )
from InverseDynamicsModel import get2DOF_J, get2DOF_M, get2DOF_C, get2DOF_dJ, get5DOF_dJ, get5DOF_C


class Controller:
    """
        Description:
        -----------
            Parent class for the controllers

    """
    def __init__( self, mj_sim, args, name ): 

        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data
        self.name     = name

        # Saving the arguments passed via ArgumentParsers
        self.mj_args  = args        

        # Get the number of actuators of the model
        self.n_act = len( self.mj_model.actuator_names )        

        # Get the number of generalized coordinate of the model
        self.nq = len( self.mj_model.joint_names )                

        # The list of controller parameters
        self.names_ctrl_pars = None

        # Get the number of geom names of the simulation
        self.n_geoms = len( self.mj_model.geom_names )        

        # Saving the name of the data as a string array for "save_data" method
        self.names_data = None

    def init( self ):
        """
            Initialize an empty list of conroller parameters
        """

        # Generate an empty arrays with data and controller parameters 
        if self.names_data is not None:
            [ setattr( self, name + "_arr", [ ] ) for name in self.names_data  ]

        if self.names_ctrl_pars is not None:
            [ setattr( self,          name, [ ] ) for name in self.names_ctrl_pars  ]

    def save_data( self ):
        """
            Update the saved, which will be defined in method "input_calc"
        """
        
        for name in self.names_data:
            val = getattr( self, name )
            getattr( self, name + "_arr" ).append( val )
        
    def export_data( self, dir_name ):
        """
            Export the data as mat file
        """
        file_name = dir_name + "/ctrl_" + self.name + ".mat"
        
        # Packing up the arrays as a dictionary
        # For the data, we simply take the transpose
        dict1 = { name + "_arr": np.transpose( getattr( self, name + "_arr" ) ) for name in self.names_data      }
        dict2 = { name: getattr( self, name )                   for name in self.names_ctrl_pars }


        scipy.io.savemat( file_name, { **dict1, **dict2 } )

    def input_calc( self, t ):
        """
            Calculating the torque input for the given time 
        """
        raise NotImplementedError

# ============================================================ #
# ================ Dynamic Motor Primitives ================== #
# ============================================================ #

class ImpedanceController( Controller ):
    """
        Description:
        -----------
            Parent class for the Impedance Controller

    """

    def __init__( self, mj_sim, args, name ): 

        super( ).__init__( mj_sim, args, name )

class JointImpedanceController( ImpedanceController ):

    """
        Description:
        ----------
            Class for a Joint (i.e., Generalized coordinate) Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mj_sim, mj_args, name ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kq", "Bq", "q0i", "q0f", "D", "ti", "amps", "omegas", "offsets" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "q0", "dq", "dq0", "ddq" , "ddq0", "q0_tmp" )

        # Generate an empty lists names of parameters
        self.init( )

        # The number of submovements
        self.n_movs = 0 

        # The number of oscillations
        self.n_movs_rhyth = 0

    def set_impedance( self, Kq: np.ndarray, Bq:np.ndarray ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( Kq      ) == self.n_act and len( Bq      ) == self.n_act 

        if self.n_act != 1: assert len( Kq[ 0 ] ) == self.n_act and len( Bq[ 0 ] ) == self.n_act 

        self.Kq = Kq
        self.Bq = Bq 
        

    def add_mov_pars( self, q0i : np.ndarray, q0f: np.ndarray, D:float, ti: float ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( q0i ) == self.n_act and len( q0f ) == self.n_act
        assert D > 0 and ti >= 0

        # If done, append the mov_parameters
        self.q0i.append( q0i )
        self.q0f.append( q0f )
        self.D.append(     D )
        self.ti.append(   ti )

        self.n_movs += 1


    def add_rhythmic_mov( self, amp: np.ndarray, offset:np.ndarray, omega:float ):

        # The center must be a 3D location
        assert len( amp ) == self.n_act and len( offset ) == self.n_act
        assert omega > 0 

        self.amps.append( amp )
        self.omegas.append( omega )
        self.offsets.append( offset )
        self.n_movs_rhyth += 1

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We implement the controller. 
                The controller generates torque with the following equation 

                tau = K( q0 - q ) + B( dq0 - dq ) + tau_G 

                (d)q0: The zero-torque trajectory, which follows a minimum-jerk profile. 
                 (d)q: current angular position (velocity) of the robot

            Arguments
            ---------
                t: The current time of the simulation. 
        """
        assert self.Kq is not None and self.Bq is not None
        assert self.n_movs >= 1 or self.n_movs_rhyth >= 1

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q   = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq  = np.copy( self.mj_data.qvel[ : self.n_act ] )
        self.ddq = np.copy( self.mj_data.qacc[ : self.n_act ] )        
 
        self.q0   = np.zeros( self.n_act )
        self.dq0  = np.zeros( self.n_act )
        self.ddq0 = np.zeros( self.n_act )        

        self.q0_tmp   = np.zeros( self.n_act )

        if self.n_movs != 0:    
            for i in range( self.n_movs ):
                for j in range( self.n_act ):
                    tmp_q0, tmp_dq0, tmp_ddq0 = min_jerk_traj( t, self.ti[ i ],  self.q0i[ i ][ j ], self.q0f[ i ][ j ], self.D[ i ] )

                    self.q0_tmp[ j ] += tmp_q0

                    self.q0[ j ]   += tmp_q0 
                    self.dq0[ j ]  += tmp_dq0
                    self.ddq0[ j ] += tmp_ddq0

        if self.n_movs_rhyth != 0: 
            for i in range( self.n_movs_rhyth ):
                for j in range( self.n_act ):
                    self.q0[  j ]  +=   self.amps[ i ][ j ] * np.sin( self.omegas[ i ] * t + self.offsets[ i ][ j ] )
                    self.dq0[ j ]  +=   self.amps[ i ][ j ] * np.cos( self.omegas[ i ] * t + self.offsets[ i ][ j ] ) * self.omegas[ i ]
                    self.ddq0[ j ] += - self.amps[ i ][ j ] * np.sin( self.omegas[ i ] * t + self.offsets[ i ][ j ] ) * self.omegas[ i ] ** 2 

        tau_imp   = self.Kq @ ( self.q0 - self.q ) + self.Bq @ ( self.dq0 - self.dq )
        self.tau  = tau_imp

        if self.mj_args.is_save_data: self.save_data( )

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau

                
    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

class CartesianImpedanceController( ImpedanceController ):

    def __init__( self, mj_sim, mj_args, name ):
        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kp", "Bp", "p0i", "p0f", "D", "ti", "amps", "omegas", "offsets", "centers" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "dq", "J", "p0", "dp0", "p", "dp" )

        # Generate an empty lists names of parameters
        self.init( )

        # The number of submovements
        self.n_movs = 0 

        # Set rhythmic movement as false
        self.n_movs_rhythmic = 0

    def set_impedance( self, Kp: np.ndarray, Bp:np.ndarray ):

        # Regardless of planar/spatial robot, DOF = 3
        assert len( Kp      ) == 3 and len( Bp      ) == 3
        assert len( Kp[ 0 ] ) == 3 and len( Bp[ 0 ] ) == 3

        self.Kp = Kp
        self.Bp = Bp 

    def add_mov_pars( self, p0i : np.ndarray, p0f: np.ndarray, D:float, ti: float ):

        # Make sure the given input is 3 dimension
        # For Planar robot, all we need is 2, but for generalization.
        assert len( p0i ) == 3 and len( p0f ) == 3
        assert D > 0 and ti >= 0

        # If done, append the mov_parameters
        self.p0i.append( p0i )
        self.p0f.append( p0f )
        self.D.append(     D )
        self.ti.append(   ti )

        self.n_movs += 1

    def add_rhythmic_mov( self, amp: float, center: np.ndarray, omega:float ):

        assert amp > 0 and omega > 0 

        self.amps.append( amp )
        self.omegas.append( omega )
        self.centers.append( center )
        self.n_movs_rhythmic += 1

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We implement the controller. 
                The controller generates torque with the following equation 

                tau = J^T( Kp( p0 - L(q) ) + Bp( dp0 - Jdq )  )

                L(q) is the forward kinematics map of the end-effector

            Arguments
            ---------
                t: The current time of the simulation. 
        """
        assert self.Kp is not None and self.Bp is not None
        assert self.n_movs >= 1 or self.n_movs_rhythmic >= 1

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q  = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq = np.copy( self.mj_data.qvel[ : self.n_act ] )

        # Get the Jacobian of the end-effector
        # The Jacobian is 3-by-nq, although we only need the first two components
        self.J    = np.copy( self.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )

        # Get the end-effector trajectories
        self.p  = np.copy( self.mj_data.get_site_xpos(  "site_end_effector" ) )
        self.dp = np.copy( self.mj_data.get_site_xvelp( "site_end_effector" ) )
 
        # The zero-force traejctory (3D)
        self.p0  = np.zeros( 3 )
        self.dp0 = np.zeros( 3 )

        if self.n_movs != 0:
            for i in range( self.n_movs ):
                for j in range( 3 ):
                    tmp_p0, tmp_dp0, _ = min_jerk_traj( t, self.ti[ i ], self.p0i[ i ][ j ], self.p0f[ i ][ j ], self.D[ i ] )

                    self.p0[ j ]  += tmp_p0 
                    self.dp0[ j ] += tmp_dp0

        if self.n_movs_rhythmic != 0:
            for i in range( self.n_movs_rhythmic ):
                self.p0[ 0 ] += self.centers[ i ][ 0 ] + self.amps[ i ] * np.sin( self.omegas[ i ] * t )
                self.p0[ 1 ] += self.centers[ i ][ 1 ] + self.amps[ i ] * np.cos( self.omegas[ i ] * t )
                self.p0[ 2 ] += 0

                self.dp0[ 0 ] +=   self.amps[ i ] * self.omegas[ i ] * np.cos( self.omegas[ i ] * t )
                self.dp0[ 1 ] += - self.amps[ i ] * self.omegas[ i ] * np.sin( self.omegas[ i ] * t )
                self.dp0[ 2 ] += 0

        self.tau  = self.J.T @ ( self.Kp @ ( self.p0 - self.p ) + self.Bp @ (self.dp0 - self.dp ) )

        if self.mj_args.is_save_data: self.save_data( )

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau

    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

class CartesianImpedanceControllerObstacle( ImpedanceController ):

    def __init__( self, mj_sim, mj_args, name, obs_pos ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "k", "n", "obs_pos" )

        # The name of the controller parameters 
        self.names_data = ( "t", "F" )        

        # Generate an empty lists names of parameters
        self.init( )

        # The position of the obstacle
        # [TODO] [2022.10.22] [Moses C. Nah]
        # We can make this as a "moving obstacle"
        self.obs_pos = obs_pos        

    def set_impedance( self, k ):

        assert k >= 0 

        self.k = k

    def set_order( self, n : int ):

        assert n >= 1

        self.n = n        

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We Modified the Cartesian Controller

            Arguments
            ---------
                t: The current time of the simulation. 
        """

        assert self.k is not None

        # Save the current time 
        self.t = t 

        # Get the current robot end-effector's Jacobian, position and velocity
        J    = self.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 )
        xEE  = self.mj_data.get_site_xpos(  "site_end_effector" )

        # The displacement between the current position and obstacle 
        delta_x = self.obs_pos - xEE

        # Getting the radial displacement 
        r2 = np.linalg.norm( delta_x , ord = 2 ) 

        # The magnitude and direction of the repulsive force 
        self.F = - self.k * ( ( 1 / r2 ) ** self.n )  * delta_x 

        self.tau  = J.T @ self.F

        #     (1) index array       (3) The tau value
                # FILL IN
        return  np.arange( self.n_act ), self.tau

    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

class CartesianImpedanceControllerModulated( CartesianImpedanceController ):
    
    def __init__( self, mj_sim, mj_args, name, Lmax ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kp", "Bp", "p0i", "p0f", "D", "ti", "amps", "omegas", "offsets", "centers", "Lmax" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "dq", "J", "p0", "dp0", "p", "dp", "pot", "kin", "my_lambda" )

        # The lambda function used for the controller 
        # lambda is already reserved for python
        self.my_lambda = 0

        # Generate an empty lists names of parameters
        self.init( )
        
        # Lmax is the maximum allowed energy of the controller 
        self.Lmax = Lmax

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We Modified the Cartesian Controller

            Arguments
            ---------
                t: The current time of the simulation. 
        """

        assert self.Kp is not None and self.Bp is not None
        assert self.n_movs >= 1

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q  = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq = np.copy( self.mj_data.qvel[ : self.n_act ] )

        # Get the Jacobian of the end-effector
        # The Jacobian is 3-by-nq, although we only need the first two components
        self.J    = np.copy( self.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )

        # Get the end-effector trajectories
        self.p  = np.copy( self.mj_data.get_site_xpos(  "site_end_effector" ) )
        self.dp = np.copy( self.mj_data.get_site_xvelp( "site_end_effector" ) )
 
        # The zero-force traejctory (3D)
        self.p0  = np.zeros( 3 )
        self.dp0 = np.zeros( 3 )

        for i in range( self.n_movs ):
            for j in range( 3 ):
                tmp_p0, tmp_dp0, _ = min_jerk_traj( t, self.ti[ i ], self.p0i[ i ][ j ], self.p0f[ i ][ j ], self.D[ i ] )

                self.p0[ j ]  += tmp_p0 
                self.dp0[ j ] += tmp_dp0

        # Calculate lambda here!
        # For this, we need to calculate the energy of the robot
        # The elastic energy from the 

        nq = self.mj_sim.nq
        Mtmp = np.zeros( nq * nq )
        functions.mj_fullM( self.mj_sim.mj_model, Mtmp, self.mj_sim.mj_data.qM )
        M = np.copy( Mtmp.reshape( nq, -1 ) )

        # Potential and Kinetic energy
        self.pot = 1/2 * ( self.p0 - self.p ).T @ self.Kp @ ( self.p0 - self.p )
        self.kin = 1/2 * self.dq.T @ M @ self.dq
        self.Lc = self.pot + self.kin

        self.my_lambda = 1 if self.Lc <= self.Lmax else max( 0, 1/self.pot * ( self.Lmax - self.kin ) )

        self.tau  = self.J.T @ ( self.Kp @ ( self.p0 - self.p ) + self.Bp @ (self.dp0 - self.dp ) )
        self.tau *= self.my_lambda

        if self.mj_args.is_save_data: self.save_data( )

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau



# ============================================================ #
# ============= Dynamic Movement Primitives ================== #
# ============================================================ #

class DMPJointController2DOF( Controller ):

    def __init__( self, mj_sim, mj_args, name ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "q_command", "dq_command", "ddq_command"  )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "dq", "ddq", "p", "dp" )

        # Generate an empty lists names of parameters
        self.init( )

    def set_traj( self, q, dq, ddq ):

        # The joint trajectories that the controller aims to follow    
        self.q_command   =   q
        self.dq_command  =  dq
        self.ddq_command = ddq
    
    def input_calc( self, t ):

        assert self.q_command   is not None
        assert self.dq_command  is not None
        assert self.ddq_command is not None
        
        # The current time must be changed to the number of steps 
        # The number of steps only matter, hence the variable passed should match
        assert self.mj_sim.n_steps == round( t/self.mj_sim.dt )
        
        # The inverse dynamics model
        n = self.mj_sim.n_steps

        # The current time
        self.t = t

        # Get the current angular position and velocity of the robot arm only
        self.q   = np.copy( self.mj_data.qpos[ :self.n_act ] )
        self.dq  = np.copy( self.mj_data.qvel[ :self.n_act ] )
        self.ddq = np.copy( self.mj_data.qacc[ :self.n_act ] )        

        # Get the current angular position and velocity of the robot arm only
        self.p   = np.copy( self.mj_data.get_site_xpos(  "site_end_effector" ) )
        self.dp  = np.copy( self.mj_data.get_site_xvelp( "site_end_effector" ) )

        # The inverse dynamics model 
        self.tau = get2DOF_M( self.q_command[ :, n ]  ) @ self.ddq_command[ :, n ] + \
                   get2DOF_C( self.q_command[ :, n ], self.dq_command[ :, n ] ) @ self.dq_command[ :, n ]

        if self.mj_args.is_save_data: self.save_data( )

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau


class DMPTaskController2DOF( Controller ):

    def __init__( self, mj_sim, mj_args, name ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "p_command", "dp_command", "ddp_command"  )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "dq", "ddq", "p", "dp", "J", "dJ" , "q_actual", "dq_actual", "ddq_actual" )

        # Generate an empty lists names of parameters
        self.init( )

    def set_traj( self, p, dp, ddp ):

        # The joint trajectories that the controller aims to follow    
        self.p_command   =   p
        self.dp_command  =  dp
        self.ddp_command = ddp
    
    def input_calc( self, t ):

        assert self.p_command   is not None
        assert self.dp_command  is not None
        assert self.ddp_command is not None
        
        # The current time must be changed to the number of steps 
        # The number of steps only matter, hence the variable passed should match
        assert self.mj_sim.n_steps == round( t/self.mj_sim.dt )
        
        # The inverse dynamics model
        n  = self.mj_sim.n_steps
        px = self.p_command[ 0, n ]
        py = self.p_command[ 1, n ]
        
        # Solve the inverse kinematics 
        q2 = np.pi - np.arccos( 0.5 * ( 2 - px ** 2 - py ** 2  ) )
        q1 = np.arctan2( py, px ) - q2/2 

        # The end-effector position and velocity
        self.p  = np.copy( self.mj_data.get_site_xpos(  "site_end_effector" ) )
        self.dp = np.copy( self.mj_data.get_site_xvelp( "site_end_effector" ) )

    
        self.t  = t
        
        # The joint positions
        self.q   = np.array( [ q1, q2 ] )
        self.q_actual = np.copy( self.mj_data.qpos[ :self.n_act ])

        # The joint velocities
        self.J   = get2DOF_J( self.q )
        self.dq  = np.linalg.inv( self.J ) @ self.dp_command[ :2, n ]
        self.dq_actual = np.copy( self.mj_data.qvel[ :self.n_act ])

        # The joint accelerations
        self.dJ = get2DOF_dJ( self.q, self.dq )
        self.ddq = np.linalg.inv( self.J ) @ ( self.ddp_command[ :2, n ] - self.dJ @ self.dq )
        self.ddq_actual = np.copy( self.mj_data.qacc[ :self.n_act ])

        # The torque array
        self.tau = get2DOF_M( self.q ) @ self.ddq + get2DOF_C( self.q, self.dq ) @ self.dq

        # [ADDED] Also superimpose a low PD control
        self.q_real  = np.copy( self.mj_sim.mj_data.qpos[ : ] )
        self.dq_real = np.copy( self.mj_sim.mj_data.qvel[ : ] )
        # Superimpose a low-gain PD control
        self.tau += 50 * ( self.q - self.q_real ) + 30 * ( self.dq - self.dq_real )

        if self.mj_args.is_save_data: self.save_data( )

        # (1) index array (3) The tau value
        return  np.arange( self.n_act ), self.tau


class DMPTaskController5DOF( Controller ):

    def __init__( self, mj_sim, mj_args, name ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "p_command", "dp_command", "ddp_command"  )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "dq", "ddq", "p", "dp", "J", "dJ", "dpr", "ddpr", "dqr", "ddqr" )

        # Generate an empty lists names of parameters
        self.init( )

    def set_traj( self, p, dp, ddp ):

        # The joint trajectories that the controller aims to follow    
        self.p_command   =   p
        self.dp_command  =  dp
        self.ddp_command = ddp
    
    def input_calc( self, t ):

        assert self.p_command   is not None
        assert self.dp_command  is not None
        assert self.ddp_command is not None
        
        # The current time must be changed to the number of steps 
        # The number of steps only matter, hence the variable passed should match
        assert self.mj_sim.n_steps == round( t/self.mj_sim.dt )

        n = self.mj_sim.n_steps

        self.t = t
        
        # Get current end-effector position and velocity 
        self.p  = np.copy( self.mj_sim.mj_data.get_site_xpos(   "site_end_effector" ) )
        self.dp = np.copy( self.mj_sim.mj_data.get_site_xvelp(  "site_end_effector" ) )

        # Get the current robot's joint position/velocity trajectories
        self.q   = np.copy( self.mj_sim.mj_data.qpos[ : ] )
        self.dq  = np.copy( self.mj_sim.mj_data.qvel[ : ] )
        self.ddq = np.copy( self.mj_sim.mj_data.qacc[ : ] )

        # The Jacobians
        self.J  = np.copy( self.mj_sim.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )
        jac_pinv = np.linalg.pinv( self.J )

        # Get the time-derivative of the Jacobian
        self.dJ   = get5DOF_dJ( self.q, self.dq )

        # Define the reference p trajectory 
        self.dpr  =  self.dp_command[ :, n ] + 80 * np.eye( 3 ) @ (  self.p_command[ :, n ] - self.p  )
        self.ddpr = self.ddp_command[ :, n ] + 80 * np.eye( 3 ) @ ( self.dp_command[ :, n ] - self.dp )

        # The dq/ddq reference trajectory 
        self.dqr  = jac_pinv @ self.dpr 
        self.ddqr = jac_pinv @ ( self.ddpr - self.dJ @ self.dq )

        # The mass/Coriolis matrices
        nq = self.mj_sim.nq
        Mtmp = np.zeros( nq * nq )
        functions.mj_fullM( self.mj_sim.mj_model, Mtmp, self.mj_sim.mj_data.qM )
        self.M = np.copy( Mtmp.reshape( nq, -1 ) )
        self.C = get5DOF_C( self.q, self.dq )

        self.tau = self.M @ self.ddqr + self.C @ self.dqr - 100 * np.eye( nq ) @ ( self.dq - self.dqr )

        if self.mj_args.is_save_data: self.save_data( )

        # (1) index array (3) The tau value
        return  np.arange( self.n_act ), self.tau

# class DMPTaskControllerObstacle( Controller ):
