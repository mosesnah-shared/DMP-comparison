import math
import scipy.io

import numpy             as np
import matplotlib.pyplot as plt

from CanonicalSystem import CanonicalSystem 
from BasisFunctions  import BasisFunctions

class DynamicMovementPrimitives:
    """
        Dynamic Movement Primitives
        
        Both the Transformation System and Nonlinear Forcing Term is Derived Here
    """

    def __init__( self, name: str, mov_type:str, cs, n_bfs = 50, alpha_z = 24.0, beta_z = 6.0 ):

        # Define the Transformation System 
        # Equation 2.1 of REF
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.

        # Name of this DMP
        self.name = name

        # Assert that mov_type is either discrete or rhythmic
        assert mov_type in [ "discrete", "rhythmic" ]
        self.mov_type = mov_type

        # Usually, beta_z = alpha_z/4 to make the transformation system critically damped
        # Hence, setting the default values of alpha_z, beta_z as 24.0, 6.0, respectively.
        self.alpha_z = alpha_z
        self.beta_z  = beta_z

        # The canonical system of the Dynamic Movement Primitives
        # The canonical system MUST be defined for the Dynamic Movement Primitives
        # cs should NOT be empty
        assert cs is not None 
        self.cs = cs

        # Tau of the canonical system and the DMP should match
        self.tau = self.cs.tau 

        # The basis functions for the nonlinear forcing term
        self.basis_functions = BasisFunctions( mov_type = mov_type, n_bfs = n_bfs, cs = cs )

        # The weights of the basis function, set as zero initially
        self.weights = np.zeros( self.basis_functions.n_bfs )

    def step( self, g, y, z, f, dt ):
        """
            Get the next y and z value with the dt step.

            We set g as an input, since there are cases when g is a time-changing variable.

            Get also the current time of the simulation
        """
        dy = z / self.tau
        dz = ( self.alpha_z * self.beta_z * ( g - y ) - self.alpha_z * z + f ) / self.tau

        y_new = dy * dt + y
        z_new = dz * dt + z

        return y_new, z_new, dy, dz

    def imitation_learning( self, t_des, y_des, dy_des, ddy_des ):
        """
            Learning the weights, center position and height of the basis functions
        """

        # Assert a 1D array and same length
        assert t_des.ndim == 1 and y_des.ndim == 1 and dy_des.ndim == 1 and ddy_des.ndim == 1
        assert len( t_des ) == len( y_des ) and len( t_des ) == len( dy_des ) and len( t_des ) == len( ddy_des )

        # Save the desired trajectory and the number of basis function s
        self.t_des   =   t_des
        self.y_des   =   y_des
        self.dy_des  =  dy_des
        self.ddy_des = ddy_des

        # Some parameters must be defined beforehand
        y0   = y_des[  0 ]
        goal = y_des[ -1 ] if self.mov_type == "discrete" else 0.5 * ( max( y_des ) + min( y_des ) )

        # Calculate the target forces based on the desired y, dy and ddy
        # Equation 2.12 of [REF]
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
        self.f_target = ( self.tau ** 2 ) * self.ddy_des + self.alpha_z * self.tau * self.dy_des  + self.alpha_z * self.beta_z * ( self.y_des - goal ) 

        # The xi array of Eq. 2.14 of [REF]
        # [REF] IBID
        s_arr = self.cs.get_value( self.t_des ) * ( goal - y0 ) if self.mov_type == "discrete" else np.ones_like( self.t_arr )
            
        for i in range( self.basis_functions.n_bfs ):
            gamma = self.basis_functions.calc_ith_activation( i, self.cs.get_value( self.t_des ) ) 

            # In case if denominator zero, then set value as zero
            self.weights[ i ] = np.sum( s_arr * gamma * self.f_target ) / np.sum( s_arr * gamma * s_arr ) if np.sum( s_arr * gamma * s_arr ) != 0 else 0


    def integrate( self, y0, z0, g, dt, N ):
        """
            Integrating the tranformation system

            Time step is dt, number of time step is N
        """
        self.y_arr  = np.zeros( N )
        self.z_arr  = np.zeros( N )
        self.dy_arr = np.zeros( N )        
        self.dz_arr = np.zeros( N )

        # The initial value 
        self.y_arr[ 0 ] = y0
        self.z_arr[ 0 ] = z0

        self.t_arr = dt * np.arange( N )

        for i in range( N - 1 ):

            # Get the current time of the time_arr
            t = self.t_arr[ i ]

            # Get the current canonical function value
            s = self.cs.get_value( t )
            f = self.basis_functions.calc_nonlinear_forcing_term( s, self.weights )
        
            # If discrete movement, multiply (g-y0) and s on the value 
            f *= s * ( g - y0 ) if self.mov_type == "discrete" else 1

            self.y_arr[ i + 1 ], self.z_arr[ i + 1 ], self.dy_arr[ i ], self.dz_arr[ i ] = self.step( g, self.y_arr[ i ], self.z_arr[ i ] ,f, dt )

        # Copy the final elements
        self.dy_arr[ -1 ] = self.dy_arr[ -2 ]
        self.dz_arr[ -1 ] = self.dz_arr[ -2 ]

        return np.copy( self.t_arr ), np.copy( self.y_arr ), np.copy( self.z_arr ), np.copy( self.dy_arr ), np.copy( self.dz_arr )

    def save_mat_data( self, dir2save ):
        """
            Basic code for saving the data of this dmp
            This saves the data in .mat form 
        """

        dict  = { "mov_type" : self.mov_type,  "weights": self.weights, "alpha_z": self.alpha_z,  "beta_z": self.beta_z, 
                   "t_des": self.t_des, "y_des": self.y_des, "dy_des": self.dy_des, "ddy_des": self.ddy_des,
                    "centers": self.basis_functions.centers, "heights": self.basis_functions.heights,
                  "t_arr": self.t_arr, "y_arr": self.y_arr, "z_arr": self.z_arr, 
                  "dy_arr": self.dy_arr, "dz_arr": self.dz_arr, 
                  "tau": self.cs.tau, "alpha_s": self.cs.alpha_s }

        scipy.io.savemat( dir2save + "/" + self.name + ".mat", { **dict } )    




if __name__ == "__main__":

    # Quick Test of Dynamic Movement Primitives 
    # The type of movement
    mov_type = "discrete"
    
    # Define the canonical system
    cs = CanonicalSystem( mov_type = mov_type )

    n_bfs = 20

    dmp = DynamicMovementPrimitives( mov_type = mov_type, cs = cs, n_bfs = n_bfs, alpha_z = 10, beta_z = 2.5, tau = 1.0 )

    # Train the movement as minimum jerk trajectory 
    P  = 100
    dt = 1.0/P

    t_arr = dt * np.arange( P )
    pi = 0.
    pf = 1.
    D  = 1.

    y_des   =              pi + ( pf - pi ) * ( 10. * ( t_arr/D ) ** 3 -  15. * ( t_arr/D ) ** 4 +   6. * ( t_arr/D ) ** 5 )
    dy_des  =      ( 1. / D ) * ( pf - pi ) * ( 30. * ( t_arr/D ) ** 2 -  60. * ( t_arr/D ) ** 3 +  30. * ( t_arr/D ) ** 4 )
    ddy_des = ( 1. / D ** 2 ) * ( pf - pi ) * ( 60. * ( t_arr/D ) ** 1 - 180. * ( t_arr/D ) ** 2 + 120. * ( t_arr/D ) ** 3 )

    # Imitation learning of this trajectory 
    dmp.imitation_learning( t_arr, y_des, dy_des, ddy_des )
    t_arr2, y_arr, z_arr, _, _ = dmp.integrate( 0, 0, pf, 0.001, D/0.001 )

    plt.plot( t_arr2, y_arr )
    plt.plot( t_arr2, z_arr )

    plt.plot( t_arr, y_des  ) 
    plt.plot( t_arr, dy_des )    

    plt.show( )