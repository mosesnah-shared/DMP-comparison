import numpy as np
import matplotlib.pyplot as plt

from CanonicalSystem import CanonicalSystem 

class BasisFunctions:
    """ 
        The Basis Function of the Nonlinear Forcing term

        Discrete Movement: Gaussian  Basis Function, Equation 2.4 of [REF]
        Rhythmic Movement: von Mises Basis Function, Equation 2.7 of [REF]
        [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
    """    

    def __init__( self, mov_type:str, n_bfs:int, cs ):
        """
            alpha_s is the gain of the canonical system 
            This value is required for discrete movement 
        """

        assert mov_type in [ "rhythmic", "discrete" ]

        # Cannot use only one basis function for the learning.
        assert n_bfs >= 2   

        self.mov_type = mov_type    
        self.n_bfs    = n_bfs

        # The Canonical system 
        self.cs = cs 

        # Depending on the movement type, define the heights and centers 
        # of the basis functions
        if self.mov_type == "discrete":

            # The center and width of the Gaussian basis function is simply 
            # Page 5 of [REF]
            # [REF] Dynamic Movement Primitives in Robotics: A Tutorial Survey
            self.centers = np.array( [ np.exp( -self.cs.alpha_s * i / ( self.n_bfs - 1 ) ) for i in range( self.n_bfs ) ] )
            self.heights = 1.0 / ( np.diff( self.centers ) ** 2 )

            # Append the final value again 
            self.heights = np.append( self.heights, self.heights[ -1 ] )

            # The lambda basis function of the discrete movement is a Gaussian Function
            self.basis_func = lambda hi, ci, s : np.exp( - hi * ( s - ci ) ** 2 )

        # If rhythmic movement, von-Mises
        else:

            self.centers = np.linspace( 0, 2 * np.pi, self.n_bfs )
            self.heights = self.n_bfs * np.ones( self.n_bfs )

            # The lambda basis function of the discrete movement is a von-Mises Function
            self.basis_func = lambda hi, ci, s : np.exp( hi * ( np.cos( s - ci ) - 1 ) )     

    def calc_ith_activation( self, i, s ):
        """
            Calculate the activation of the i-th basis function of the time arr
            The index starts/ends at 0/self.n_bfs-1, respectively.

            The s can either be an array or a scalar.
        """
    
        assert i >= 0 and i <= self.n_bfs - 1

        if   isinstance( s, ( list, tuple, np.ndarray ) ): assert s.all( ) >= 0
        elif np.isscalar( s ): assert s >= 0

        return self.basis_func( self.heights[ i ], self.centers[ i ], s ) 

    def calc_activation( self, s ):
        """
            Calculate the activations of the n_bfs at value s
            For this, s must be a scalar, and the return array is sized as n_bfs.
        """
        assert np.isscalar( s ) and s >= 0

        # In case if the activation is infitesimal to zero, then simply set the value as zero, else, not
        return np.array( [ self.calc_ith_activation( i, s ) for i in np.arange( self.n_bfs ) ] )

    def calc_total_activation( self, s ):
        """
            Calculate the activations of the n_bfs at value s
            For this, s can either be a scalar or s an array
        """
        if   isinstance( s, ( list, tuple, np.ndarray ) ): assert s.all( ) >= 0
        elif np.isscalar( s ): assert s >= 0

        # In case if the activation is infinitesimal to zero, then simply set the value as zero, else, not
        return np.sum( self.calc_activation( s ) ) if np.isscalar( s ) else np.array( [ np.sum( self.calc_activation( i ) ) for i in s ] ) 

    def calc_total_activation_w_weights( self, s, weights ):
        """
            Calculate the total activation inner product with weights.
            For this, s must be a scalar
        """
        if   isinstance( s, ( list, tuple, np.ndarray ) ): assert s.all( ) >= 0
        elif np.isscalar( s ): assert s >= 0

        assert( len( weights ) == self.n_bfs )

        return np.sum( self.calc_activation( s ) * weights ) if np.isscalar( s ) else np.array( [ np.sum( self.calc_activation( i ) * weights ) for i in s ] )    
    
    def calc_nonlinear_forcing_term( self, s, weights ):

        assert( len( weights ) == self.n_bfs )

        return np.divide( self.calc_total_activation_w_weights( s, weights ), 
                                             self.calc_total_activation( s ), 
                                                    out = np.zeros_like( s ), 
                                   where=self.calc_total_activation( s )!=0 )
    

if __name__ == "__main__":
    
    mov_type = "discrete"

    cs  = CanonicalSystem( mov_type = mov_type )
    basis_functions = BasisFunctions( mov_type = mov_type, n_bfs = 10, cs = cs )
    
    t_arr = np.linspace( 0.0, 1.0, 1000 )

    for i in range( 10 ):
        plt.plot( cs.get_value( t_arr ), basis_functions.calc_ith_activation( i, cs.get_value( t_arr ) )  )

    plt.plot( cs.get_value( t_arr ), basis_functions.calc_total_activation( cs.get_value( t_arr ) ) )        
    # plt.plot( t_arr, basis_functions.calc_total_activation_w_weights( cs.get_value( t_arr ),  np.linspace( 0, 2.0, basis_functions.n_bfs ) ) )        
    # plt.plot( t_arr, basis_functions.calc_nonlinear_forcing_term( cs.get_value( t_arr ), np.linspace( 0, 2.0, basis_functions.n_bfs ) ) )        
    
    plt.show( )    
