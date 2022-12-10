import math
import numpy as np
import matplotlib.pyplot as plt

from CanonicalSystem import CanonicalSystem 
from BasisFunctions  import BasisFunctions

class DynamicMovementPrimitives:
    """
        Dynamic Movement Primitives
        
        Both the Transformation System and Nonlinear Forcing Term is Derived Here
    """

    def __init__( self, mov_type:str, cs, n_bfs = 50, alpha_z = 24.0, beta_z = 6.0, tau = 1.0 ):

        # Define the Transformation System 
        # Equation 2.1 of REF
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.

        # Assert that mov_type is either discrete or rhythmic
        assert mov_type in [ "discrete", "rhythmic" ]
        self.mov_type = mov_type

        # Usually, beta_z = alpha_z/4 to make the transformation system critically damped
        # Hence, setting the default values of alpha_z, beta_z as 24.0, 6.0, respectively.
        self.alpha_z = alpha_z
        self.beta_z  = beta_z
        self.tau     = tau 

        # The canonical system of the Dynamic Movement Primitives
        # The canonical system MUST be defined for the Dynamic Movement Primitives
        # cs should NOT be empty
        assert cs is not None 
        self.cs = cs

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

    def imitation_learning( self, t_arr, y_des, dy_des, ddy_des ):
        """
            Learning the weights, center position and height of the basis functions
        """

        # Assert a 1D array and same length
        assert t_arr.ndim == 1 and y_des.ndim == 1 and dy_des.ndim == 1 and ddy_des.ndim == 1
        assert len( t_arr ) == len( y_des ) and len( t_arr ) == len( dy_des ) and len( t_arr ) == len( ddy_des )

        # Save the desired trajectory and the number of basis function s
        self.t_arr   =   t_arr
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
        s_arr = self.cs.get_value( self.t_arr ) * ( goal - y0 ) if self.mov_type == "discrete" else np.ones_like( self.t_arr )
            
        for i in range( self.basis_functions.n_bfs ):
            gamma = self.basis_functions.calc_ith_activation( i, self.cs.get_value( self.t_arr ) ) 

            # In case if denominator zero, then set value as zero
            self.weights[ i ] = np.sum( s_arr * gamma * self.f_target ) / np.sum( s_arr * gamma * s_arr ) if np.sum( s_arr * gamma * s_arr ) != 0 else 0


    def integrate( self, y0, z0, g, dt, N ):
        """
            Integrating the tranformation system

            Time step is dt, number of time step is N
        """
        y_arr  = np.zeros( N )
        z_arr  = np.zeros( N )
        dy_arr = np.zeros( N )        
        dz_arr = np.zeros( N )

        # The initial value 
        y_arr[ 0 ] = y0
        z_arr[ 0 ] = z0

        t_arr = dt * np.arange( N )

        for i in range( N - 1 ):

            # Get the current time of the time_arr
            t = t_arr[ i ]

            # Get the current canonical function value
            s = self.cs.get_value( t )
            f = self.basis_functions.calc_nonlinear_forcing_term( s, self.weights )
        
            # If discrete movement, multiply (g-y0) and s on the value 
            f *= s * ( g - y0 ) if self.mov_type == "discrete" else 1

            y_arr[ i + 1 ], z_arr[ i + 1 ], dy_arr[ i ], dz_arr[ i ] = self.step( g, y_arr[ i ], z_arr[ i ] ,f, dt )

        # Copy the final elements
        dy_arr[ -1 ] = dy_arr[ -2 ]
        dz_arr[ -1 ] = dz_arr[ -2 ]

        return t_arr, y_arr, z_arr, dy_arr, dz_arr


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