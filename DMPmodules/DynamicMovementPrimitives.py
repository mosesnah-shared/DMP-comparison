import numpy as np
import matplotlib.pyplot as plt

from CanonicalSystem import CanonicalSystem 
from BasisFunctions  import BasisFunctions 

class DynamicMovementPrimitives:
    """
        Dynamic Movement Primitives
        
        Both the Transformation System and Nonlinear Forcing Term is Derived Here
    """

    def __init__( self, mov_type:str,  **kwargs ):

        # Define the Transformation System 
        # Equation 2.1 of REF
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.

        # Assert that mov_type is either discrete or rhythmic
        assert mov_type in [ "discrete", "rhythmic" ]
        self.mov_type = mov_type

        # Usually, beta_z = alpha_z/4 to make the transformation system critically damped
        self.alpha_z = kwargs.get( "alpha_z", 24.0 )
        self.beta_z  = kwargs.get(  "beta_z",  6.0 )

        # The time constant tau of the system
        self.tau = kwargs.get( "tau", 1.0 )

        # The weights of the basis functions
        # This is set as None initially
        self.weights = None

        # The canonical system of the Dynamic Movement Primitives 
        self.cs  = None

        # The basis functions for the nonlinear forcing term
        self.basis_functions = None

    def add_canonical_system( self, cs ):
        assert cs.mov_type == self.mov_type
        self.cs = cs

    def imitation_learning( self, t_arr, y_des, dy_des, ddy_des, n_bfs ):
        """
            Learning the weights, center position and height of the basis functions
        """

        # Assert a 1D array and same length
        assert t_arr.ndim == 1 and y_des.ndim == 1 and dy_des.ndim == 1 and ddy_des.ndim == 1
        assert len( t_arr ) == len( y_des ) and len( t_arr ) == len( dy_des ) and len( t_arr ) == len( ddy_des )

        # Assert the existence of canonical system and basis functions to fit
        assert self.cs is not None

        # Assert the number of basis functions should be more than 2
        n_bfs >= 2
        
        # Define the Basis functions 
        # Define the basis function for the nonlinear forcing term
        self.basis_functions = BasisFunctions( mov_type = mov_type, n_bfs = n_bfs, alpha_s = cs.alpha_s )

        # Save the desired trajectory and the number of basis function s
        self.t_arr   =  t_arr
        self.y_des   =  y_des
        self.dy_des  = dy_des
        self.ddy_des = ddy_des

        # The parameter tau must be adjusted to the duration of the demonstration.
        # Page 342 of REF
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
        self.tau = t_arr[ -1 ]

        # Setting-up the initial and goal positions of the trajectory 
        if self.mov_type == "discrete":
            y0   = y_des[ 0  ]
            goal = y_des[ -1 ]
            
        else:
            # For rhythmic movement, there must be some additional work to be done
            NotImplementedError( )

        # Calculate the target forces based on the desired y, dy and ddy
        # Equation 2.12 of [REF]
        # [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
        self.f_target = ( self.tau ** 2 ) * self.ddy_des - self.alpha_z * self.beta_z * ( goal - self.y_des ) + self.alpha_z * self.tau * self.dy_des 
        
        self.weights = np.zeros( n_bfs )

        # Setting-up the goal position and others
        if self.mov_type == "discrete":

            # The center and width of the Gaussian basis function is simply 
            # Page 5 of [REF]
            # [REF] Dynamic Movement Primitives in Robotics: A Tutorial Survey

            # The canonical system's values 
            s_arr = self.cs.get_value( self.t_arr ) * ( goal - y0 ) 
            
            for i in range( n_bfs ):
                gamma = np.array( [ self.basis_functions.calc_activation( i, self.cs.get_value( t ) ) for t in self.t_arr ] )
                self.weights[ i ] = np.sum( s_arr * gamma * self.f_target ) / np.sum( s_arr * gamma * s_arr )
            
        else:
            # For rhytmic movement, there must be some additional work to be done
            NotImplementedError( )

    def integrate( self, y0, z0, g, dt, N ):
        """
            Integrating the tranformation system
        """
        y_arr = np.zeros( N )
        z_arr = np.zeros( N )

        y_arr[ 0 ] = y0
        z_arr[ 0 ] = z0

        t_arr = dt * np.arange( N )

        for i in range( N - 1 ):

            if self.weights is None:
                f = 0 

            else:
                # Get the current time of the 
                t = t_arr[ i ]

                # Get the current canonical function value
                s = self.cs.get_value( t )

                psi_arr = np.array( [ self.basis_functions.calc_activation( j, s ) for j in np.arange( self.basis_functions.n_bfs ) ] )
                f = np.sum( self.weights * psi_arr ) / np.sum( psi_arr )
                
                # If discrete movement, multiply (g-y0) and s on the value 
                f *= s * ( g - y0 )

            dy = z_arr[ i ] / self.tau
            dz = ( self.alpha_z * self.beta_z * ( g - y_arr[ i ] ) - self.alpha_z * z_arr[ i ] + f ) / self.tau

            y_arr[ i + 1 ] = dy * dt + y_arr[ i ]
            z_arr[ i + 1 ] = dz * dt + z_arr[ i ]

        return t_arr, y_arr, z_arr 


if __name__ == "__main__":

    # Quick Test of Dynamic Movement Primitives 
    # The type of movement
    mov_type = "discrete"
    
    # Define the canonical system
    cs = CanonicalSystem( mov_type = mov_type )

    # Dynamic Movement Primitives
    dmp = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )

    # Train the movement as minimum jerk trajectory 
    N = 100 
    t_arr = 0.01 * np.arange( N )
    pi = 0.
    pf = 1.
    D  = 1.

    y_des   =              pi + ( pf - pi ) * ( 10. * ( t_arr/D ) ** 3 -  15. * ( t_arr/D ) ** 4 +   6. * ( t_arr/D ) ** 5 )
    dy_des  =      ( 1. / D ) * ( pf - pi ) * ( 30. * ( t_arr/D ) ** 2 -  60. * ( t_arr/D ) ** 3 +  30. * ( t_arr/D ) ** 4 )
    ddy_des = ( 1. / D ** 2 ) * ( pf - pi ) * ( 60. * ( t_arr/D ) ** 1 - 180. * ( t_arr/D ) ** 2 + 120. * ( t_arr/D ) ** 3 )

    # Imitation learning of this trajectory 
    dmp.add_canonical_system( cs )

    dmp.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = 100 )

    t_arr2, y_arr, z_arr = dmp.integrate( 0, 0, pf, 0.001, 1000 )

    plt.plot( t_arr2, y_arr )
    plt.plot( t_arr, y_des )
    plt.plot( t_arr2, z_arr )
    plt.plot( t_arr, dy_des )


    plt.show( )

