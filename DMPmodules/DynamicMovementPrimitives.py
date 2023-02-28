import scipy.io
import numpy             as np
import matplotlib.pyplot as plt

from CanonicalSystem import CanonicalSystem 
from BasisFunctions  import BasisFunctions

class DynamicMovementPrimitives:
    """
        Descriptions
        ------------       
            Dynamic Movement Primitives, from
            [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
        
            Both the Transformation System and Nonlinear Forcing Term is Derived Here
            Imitation Learning is also implemented here
    """

    def __init__( self, name: str, mov_type:str, cs, n_bfs = 50, alpha_z = 24.0, beta_z = 6.0 ):
        """
        Descriptions
        ------------
            Constructor of Dynamic Movement Primitives DMP
            The transformation system - Equation 2.1 of [REF] - is implemeneted
            The transformation system is a 2nd-order linear system with a nonlinear force input.      

            tau^2 ddy + alpha_z * tau * dy + alpha_z * beta_z * ( y-g ) = f( s ) 


        Parameters
        ----------
            (1) name: str
                    The name of this DMP, e.g., dmp1, dmp2

            (2) mov_type: str
                    Either "discrete" or "rhythmic" movement type.

            (3) cs : CanonicalSystem
                    The canonical system for DMP
                    Refer to "CanonicalSystem.py"

            (4) n_bfs: int (default 50)
                    The number of basis functions for the nonlinear forcing term.

            (5) alpha_z: float (default 24.0)
                    alpha_z value for the transformation system:
                    tau^2 ddy + alpha_z * tau * dy + alpha_z * beta_z * ( y-g ) = f( s ) 

            (6) beta_Z: float (default 6.0)
                    beta_z value for the transformation system:
                    tau^2 ddy + alpha_z * tau * dy + alpha_z * beta_z * ( y-g ) = f( s )             

                    Usually, beta_z = alpha_z/4 to make the transformation system critically damped

        """

        self.name = name

        assert mov_type in [ "discrete", "rhythmic" ]
        self.mov_type = mov_type

        self.alpha_z = alpha_z
        self.beta_z  = beta_z

        # The canonical system MUST be defined for the Dynamic Movement Primitives
        # cs should NOT be empty
        assert cs is not None 
        self.cs = cs

        # tau of the canonical system and the DMP should match
        self.tau = self.cs.tau 

        # The basis functions for the nonlinear forcing term
        self.basis_functions = BasisFunctions( mov_type = mov_type, n_bfs = n_bfs, cs = cs )

        # Initialization of the weights of the nonlinear forcing term.
        # [TODO] [Moses C. Nah] [2023.02.27]
        # Will be good to add a "init" method to initialize the values. 
        self.weights = np.zeros( self.basis_functions.n_bfs )

        # The desired trajectory which we aim to imitate 
        # These *_des member variables are used to conduct imitation learning.
        self.t_des   = 0
        self.y_des   = 0
        self.dy_des  = 0
        self.ddy_des = 0

        # The resulting trajectory of the transformation system.
        # In other words, we save all the state variables for the trasformation system.
        self.t_arr  = 0
        self.y_arr  = 0
        self.z_arr  = 0
        self.dy_arr = 0
        self.dz_arr = 0
        

    def step( self, g, y, z, f, dt ):
        """

        Descriptions
        ------------
            A single integration of the transformation system, given dt as a step size.
            In other words, we calculate the new y and z values of the transformation system

            [Note] [Moses C. Nah] [2023.02.27]
            Here, we set goal g as an function argument,
            since there are cases and applications when g is a time-changing variable (e.g., sequenced movements)
    
            Recall that the transformation system is defined by (Equation 1):
                tau dy = z
                tau dz = alpha_z * { beta_z * ( g - y ) - z } + f( s )
            
            In a single differential equation (Equation 2):
                tau^2 ddy + alpha_z * tau * dy + alpha_z * beta_z * ( y-g ) = f( s ) 

            
        Parameters
        ----------
            (1) g: float
                    The goal state g (Equation 1)

            (2) y: float
                    The y state (weighted position) (Equation 1)

            (3) z: float
                    The z state (weighted position) (Equation 1)            

            (4) f: float            
                    The nonlinear forcing term

            (5) dt: time step
                    The time step for the integration

        Returns
        -------
            (1) y_new: float
                    y_new = dy * dt + y

            (2) z_new: float
                    z_new = dz * dt + z

            (3) dy: float
                    The rate of change of dy (left-hand side of equation 1)

            (4) dz: float
                    The rate of change of dz (left-hand side of equation 1)
            
        """
        
        dy = z / self.tau
        dz = ( self.alpha_z * self.beta_z * ( g - y ) - self.alpha_z * z + f ) / self.tau

        y_new = dy * dt + y
        z_new = dz * dt + z

        return y_new, z_new, dy, dz

    def imitation_learning( self, t_des, y_des, dy_des, ddy_des ):
        """
        Descriptions
        ------------
            Imitation learning of DMP
            Given the desired trajectory (position, velocity, acceleration, and time array)
            We find the nonlinear forcing term which produces the desired trajectory.

            In detail, the goal is to find the forcing term f(s) which satisfies:
                tau^2 ddy_des + alpha_z * tau * dy_des + alpha_z * beta_z * ( y_des-g ) = f( s ) 

            Note that for goal g,
                For discrete movement, goal g is the final position of y_des
                For rhythmic movement, goal g is the average position of y_des
                [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.

            f(s) consists of N weights for N basis functions, the weights are learned in this method.
                
        Parameters
        ----------
            (1) t_des: float (1D array)
                    The time array of the desired trajectory to imitate

            (2) y_des: float (1D array)
                    The position of the desired trajectory to imitate

            (3) dy_des: float (1D array)
                    The velocity of the desired trajectory to imitate

            (4) ddy_des: float (1D array)
                    The acceleration of the desired trajectory to imitate

        Returns
        -------
            There are no returns for this method, because the learned weights 
            are saved directly to the self.basis_functions, 
            which are defined at the default constructor of __init__():
            
        """

        # Assert a 1D array and same length
        assert t_des.ndim == 1 and y_des.ndim == 1 and dy_des.ndim == 1 and ddy_des.ndim == 1
        assert len( t_des ) == len( y_des ) and len( t_des ) == len( dy_des ) and len( t_des ) == len( ddy_des )

        # Save the desired trajectory and the number of basis function s
        self.t_des   =   t_des
        self.y_des   =   y_des
        self.dy_des  =  dy_des
        self.ddy_des = ddy_des

        # The initial position y0 and the goal parameters must be defined beforehand
        y0   = y_des[  0 ]
        goal = y_des[ -1 ] if self.mov_type == "discrete" else 0.5 * ( max( y_des ) + min( y_des ) )

        # Calculate the target forces based on the desired y, dy and ddy
        # Equation 2.12 of [REF]
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
        self.f_target = ( self.tau ** 2 ) * self.ddy_des + self.alpha_z * self.tau * self.dy_des  + self.alpha_z * self.beta_z * ( self.y_des - goal ) 

        # The xi array of Eq. 2.14 of [REF]
        # [REF] IBID
        s_arr = self.cs.get_value( self.t_des ) * ( goal - y0 ) if self.mov_type == "discrete" else np.ones_like( self.t_des )
            
        # Learning the weights
        for i in range( self.basis_functions.n_bfs ):
            gamma = self.basis_functions.calc_ith_activation( i, self.cs.get_value( self.t_des ) ) 

            # In case if denominator zero, then set value as zero
            self.weights[ i ] = np.sum( s_arr * gamma * self.f_target ) / np.sum( s_arr * gamma * s_arr ) if np.sum( s_arr * gamma * s_arr ) != 0 else 0


    def integrate( self, y0, z0, g, dt, t0, N ):
        """
        Descriptions
        ------------
            The whole integration of the transformation system.
            Given time step of dt, we conduct N of dt time steps, i.e., total time = N * dt
                   
        Parameters
        ----------
            (1) y0: float
                    The initial value of y for the transformation system.

            (2) z0: float
                    The initial value of z for the transformation system.

            (3) g: float
                    The goal state g 

            (4) dt: float            
                    The time step for the integration

            (5) t0: float
                    The initial time for the time step
                    This is required when we want to start the movement at a specific time.

            (6) N: int
                    The number of time steps to conduct
        
        Returns
        -------
            (1) t_arr: float (1D array)

            (2) y_arr: float (1D array)

            (3) z_arr: float (1D array)
            
            (4) dy_arr: float (1D array)

            (5) dz_arr: float (1D array)

        """

        # Initialization of the arrays to learn
        self.y_arr  = np.zeros( N )
        self.z_arr  = np.zeros( N )
        self.dy_arr = np.zeros( N )        
        self.dz_arr = np.zeros( N )

        # The initial value of the transformation system
        self.y_arr[ 0 ] = y0
        self.z_arr[ 0 ] = z0

        # The time array for integration
        self.t_arr = dt * np.arange( N )

        # Conduct integration
        for i in range( N - 1 ):

            # Get the current time of the time_arr
            t = self.t_arr[ i ]

            # If time is smaller than the initial time, then just stay there
            if t < t0:
                self.y_arr[ i + 1 ] = self.y_arr[ i ]
                self.z_arr[ i + 1 ] = self.z_arr[ i ]

            else:

                # Get the current canonical function value
                s = self.cs.get_value( t - t0 )
                f = self.basis_functions.calc_nonlinear_forcing_term( s, self.weights )
            
                # If discrete movement, multiply (g-y0) and s on the value 
                f *= s * ( g - y0 ) if self.mov_type == "discrete" else 1

                # Single integration
                self.y_arr[ i + 1 ], self.z_arr[ i + 1 ], self.dy_arr[ i ], self.dz_arr[ i ] = self.step( g, self.y_arr[ i ], self.z_arr[ i ] ,f, dt )

        # Copy the final elements
        self.dy_arr[ -1 ] = self.dy_arr[ -2 ]
        self.dz_arr[ -1 ] = self.dz_arr[ -2 ]

        return np.copy( self.t_arr ), np.copy( self.y_arr ), np.copy( self.z_arr ), np.copy( self.dy_arr ), np.copy( self.dz_arr )

    def save_mat_data( self, dir2save ):
        """
        Descriptions
        ------------
            Saving the .mat file for data analysis in MATLAB
                   
        Parameters
        ----------
            (1) dir2save: str
                    The relative directory to save the .mat file

        """

        # The whole details of a single DMP
        dict  = { "mov_type" : self.mov_type,  "weights": self.weights, "alpha_z": self.alpha_z,  "beta_z": self.beta_z, 
                  "t_des": self.t_des, "y_des": self.y_des, "dy_des": self.dy_des, "ddy_des": self.ddy_des,
                  "centers": self.basis_functions.centers, "heights": self.basis_functions.heights,
                  "t_arr": self.t_arr, "y_arr": self.y_arr, "z_arr": self.z_arr, 
                  "dy_arr": self.dy_arr, "dz_arr": self.dz_arr, 
                  "tau": self.cs.tau, "alpha_s": self.cs.alpha_s }

        scipy.io.savemat( dir2save + "/" + self.name + ".mat", { **dict } )    


if __name__ == "__main__":

    # Imitation Learning for Dynamic Movement Primitives
    # Here, we show imitation learning of a discrete movement, a single minimum-jerk trajectory
    mov_type = "discrete"
    cs       = CanonicalSystem( mov_type = mov_type )
    n_bfs    = 20
    dmp      = DynamicMovementPrimitives( name = "dmp_discrete", mov_type = mov_type, cs = cs, n_bfs = n_bfs, alpha_z = 10, beta_z = 2.5 )

    # The desired trajectory to imitate, which is a minimum-jerk trajectory                                     
    P     = 100
    dt    = 1.0/P
    t_arr = dt * np.arange( P )

    # The initial, final position and duration of the minimum-jerk trajectory
    pi, pf, D = 0.0, 1.0, 1.0
    
    # The desired trajectory to imitate 
    y_des   =              pi + ( pf - pi ) * ( 10. * ( t_arr/D ) ** 3 -  15. * ( t_arr/D ) ** 4 +   6. * ( t_arr/D ) ** 5 )
    dy_des  =      ( 1. / D ) * ( pf - pi ) * ( 30. * ( t_arr/D ) ** 2 -  60. * ( t_arr/D ) ** 3 +  30. * ( t_arr/D ) ** 4 )
    ddy_des = ( 1. / D ** 2 ) * ( pf - pi ) * ( 60. * ( t_arr/D ) ** 1 - 180. * ( t_arr/D ) ** 2 + 120. * ( t_arr/D ) ** 3 )

    # Imitation learning of this trajectory 
    # A higher resolution of the time step for the integration
    dt = 0.001
    dmp.imitation_learning( t_arr, y_des, dy_des, ddy_des )
    t_arr2, y_arr, z_arr, dy_arr, dz_arr = dmp.integrate( 0, 0, pf, dt, 0, int( D/dt ) )

    # Plotting the resultsc
    fig1, ( ax1, ax2, ax3 ) = plt.subplots( nrows = 1, ncols = 3, figsize = (12, 8) )

    # The learned position, velocity and accelration
    ax1.plot(  t_arr, y_des,   lw =4, linestyle='dashed', label="des"  )
    ax1.plot( t_arr2, y_arr )    

    ax2.plot(  t_arr, dy_des,  lw =4, linestyle='dashed', label="des"  )
    ax2.plot( t_arr2, dy_arr )

    ax3.plot(  t_arr, ddy_des, lw =4, linestyle='dashed', label="des"  )
    ax3.plot( t_arr2, dz_arr  )

    ax2.set_xlabel( "$t~(sec)$")
    ax1.set_title( "Position" )  
    ax2.set_title( "Velocity" )  
    ax3.set_title( "Acceleration" )    

    ax1.legend( )
    ax2.legend( )
    ax3.legend( )
    
    
    plt.show( )