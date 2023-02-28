import numpy as np
import matplotlib.pyplot as plt

from CanonicalSystem import CanonicalSystem 

class BasisFunctions:
    """ 
        Descriptions
        ------------        
            The Basis Function of the Nonlinear Forcing term

            Discrete Movement: Gaussian  Basis Function, Equation 2.4 of [REF]
            Rhythmic Movement: von Mises Basis Function, Equation 2.7 of [REF]
            [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
    """    

    def __init__( self, mov_type:str, n_bfs:int, cs ):
        """
        Descriptions
        ------------
            Constructor of the Basis Function of the Nonlinear Forcing Term

        Parameters
        ----------
            (1) mov_type : str
                    Either "discrete" or "rhythmic" canonical system

            (2) n_bfs : int (>1)
                    The number of basis function, must be bigger than 1

            (3) cs : CanonicalSystem
                    The canonical system that is used to create the weights
                    Refer to "CanonicalSystem.py"
             
        """

        assert mov_type in [ "rhythmic", "discrete" ]
        assert n_bfs > 1 

        self.mov_type = mov_type    
        self.n_bfs    = n_bfs
        self.cs       = cs 

        # Depending on the movement type, define the heights and centers of the basis functions
        if self.mov_type == "discrete":

            # The center and height of the Gaussian basis function are calculated based on Page 5 of
            # "Dynamic Movement Primitives in Robotics: A Tutorial Survey", Matteo Saveriano et al.
            self.centers = np.array( [ np.exp( -self.cs.alpha_s * i / ( self.n_bfs - 1 ) ) for i in range( self.n_bfs ) ] )
            self.heights = 1.0 / ( np.diff( self.centers ) ** 2 )

            # Append the final value again 
            self.heights = np.append( self.heights, self.heights[ -1 ] )

            # The lambda basis function of the discrete movement is a Gaussian Function
            self.basis_func = lambda hi, ci, s : np.exp( - hi * ( s - ci ) ** 2 )

        else: # if "rhythmic"

            # The center and height of the von-Mises function are equally, uniformly distributed
            self.centers = np.linspace( 0, 2 * np.pi, self.n_bfs )
            self.heights = self.n_bfs * np.ones( self.n_bfs )

            # The lambda basis function of the discrete movement is a von-Mises Function
            self.basis_func = lambda hi, ci, s : np.exp( hi * ( np.cos( s - ci ) - 1 ) )     

    def calc_ith_activation( self, i, s ):
        """
        Descriptions
        ------------
            Let the i-th basis function be fi. Then 
            This function returns fi( s ), the activation of the i-th basis function at value s, 
            where s is the value of the canonical system.

            The index starts/ends at 0/n_bfs-1, respectively.
            
        Parameters
        ----------
            (1) i: int ( i>=0 and i<= n_bfs-1 )
                    the i-th basis function, fi(  )

            (2) s: float (scalar or array)
                    The value of the canonical system, s(t)

        Returns
        -------
            fi( s ): float (scalar or array)
                    

        """
    
        assert i >= 0 and i <= self.n_bfs - 1

        # Impose that s is a positive value.
        if   isinstance( s, ( list, tuple, np.ndarray ) ): assert s.all( ) >= 0
        elif np.isscalar( s ): assert s >= 0

        return self.basis_func( self.heights[ i ], self.centers[ i ], s ) 

    def calc_activation( self, s ):
        """
        Descriptions
        ------------
            Calculate the activations of the n_bfs at value s
            Given n basis functions of f1( s ), f2( s ), ... fn( s ), this method returns an 
            array of [ f1( s ), f2( s ), ..., fn( s ) ]
            For this, s must be a scalar, and the return array is sized as n_bfs.

        Parameters
        ----------
            (1) s: float scalar 
                   The value of the canonical system, s(t)

        Returns
        -------
            [ f1( s ), f2( s ), ..., fn( s ) ]: float array, where n is the number of basis functions

        """

        assert np.isscalar( s ) and s >= 0

        return np.array( [ self.calc_ith_activation( i, s ) for i in np.arange( self.n_bfs ) ] )

    def calc_total_activation( self, s ):
        """

        Descriptions
        ------------
            Calculate the "sum" of activations of the n_bfs at value s
            Given n basis functions of f1( s ), f2( s ), ... fn( s ), this method returns a value of
            f1( s ) + f2( s ) + ... + fn( s )

        Parameters
        ----------
            (1) s: float (scalar or array)
                   The value of the canonical system, s(t).
                   s can be an array - s1, s2, ..., sm
    
        Returns
        -------
            If we define fsum( ) = f1( ) + f2( ) + ... + fn(  ), then
            [ fsum( s1 ), fsum( s2 ), ..., fsum( sm ) ]: float array, where m is the size of the s_arr                
            
        """
        if   isinstance( s, ( list, tuple, np.ndarray ) ): assert s.all( ) >= 0
        elif np.isscalar( s ): assert s >= 0

        return np.sum( self.calc_activation( s ) ) if np.isscalar( s ) else np.array( [ np.sum( self.calc_activation( i ) ) for i in s ] ) 

    def calc_total_activation_w_weights( self, s, weights ):
        """

        Descriptions
        ------------
            Calculate the "weighted sum" of activations of the n_bfs at value s
            Given n basis functions of f1( s ), f2( s ), ... fn( s ), with n weights w1, w2, ...wn,  this method returns a value of
            w1*f1( s ) + w2*f2( s ) + ... + wn*fn( s )
            
        Parameters
        ----------
            (1) s: float (scalar or array)
                   The value of the canonical system, s(t).
                   s can be an array - s1, s2, ..., sm
            
            (2) weights: float (array)
                   The weights to be summed.
                   The number of weights should match the number of basis functions
    
        Returns
        -------
            If we define f_wsum( ) = w1*f1( ) + w2*f2( ) + ... + wn*fn(  ), then
            [ f_wsum( s1 ), f_wsum( s2 ), ..., f_wsum( sm ) ]: float array, where m is the size of the s_arr                
            

        """
        if   isinstance( s, ( list, tuple, np.ndarray ) ): assert s.all( ) >= 0
        elif np.isscalar( s ): assert s >= 0

        assert( len( weights ) == self.n_bfs )

        return np.sum( self.calc_activation( s ) * weights ) if np.isscalar( s ) else np.array( [ np.sum( self.calc_activation( i ) * weights ) for i in s ] )    
    
    def calc_nonlinear_forcing_term( self, s, weights ):

        """
        Descriptions
        ------------
            Calculate the "weighted sum" of activations divided by the "sum of activations", of the n_bfs at value s
            Given n basis functions of f1( s ), f2( s ), ... fn( s ), with n weights w1, w2, ...wn,  this method returns a value of
                       w1*f1( s ) + w2*f2( s ) + ... + wn*fn( s )    
            return  =   -----------------------------------------
                          f1( s ) +    f2( s ) + ... +    fn( s )  
        
        Parameters
        ----------
            (1) s: float (scalar or array)
                   The value of the canonical system, s(t).
                   s can be an array - s1, s2, ..., sm
            
            (2) weights: float (array)
                   The weights to be summed.
                   The number of weights should match the number of basis functions
    
        Returns
        -------
            If we define f as:
        
                       w1*f1( s ) + w2*f2( s ) + ... + wn*fn( s )    
            f( s )  =  ------------------------------------------
                          f1( s ) +    f2( s ) + ... +    fn( s ), then
            
            [ f( s1 ), f( s2 ), ..., f( sm ) ]: float array, where m is the size of the s_arr         

        """

        assert( len( weights ) == self.n_bfs )

        # In case if divided by zero, i.e., f1( s ) + f2( s ) + ... + fn( s ) = 0, then 
        # set the output value as zero.

        return np.divide( self.calc_total_activation_w_weights( s, weights ), 
                                             self.calc_total_activation( s ), 
                                                    out = np.zeros_like( s ), 
                               where = self.calc_total_activation( s ) != 0 ) 
    

if __name__ == "__main__":
    

    # ======================================================== #
    # =============== DISCRETE BASIS FUNCTIONS =============== #
    # ======================================================== #    
    n_bfs = 10
    t_arr = np.linspace( 0.0, 1.0, 1000 )
    cs1   = CanonicalSystem( mov_type = "discrete" )
    basis_functions1 = BasisFunctions( mov_type = "discrete", n_bfs = n_bfs, cs = cs1 )

    # Creating a subplot    
    fig1, ax1 = plt.subplots( figsize = (8, 4) )

    # Plot the 10 Basis Functions
    for i in range( n_bfs ):
        ax1.plot( cs1.get_value( t_arr ), basis_functions1.calc_ith_activation( i, cs1.get_value( t_arr ) )  )

    ax1.set_title( str( n_bfs ) + " Discrete Basis Functions" )


    # ======================================================== #
    # =============== RHYTHMIC BASIS FUNCTIONS =============== #
    # ======================================================== #    
    n_bfs = 10
    t_arr = np.linspace( 0.0, 2*np.pi-0.01, 1000 )
    cs2   = CanonicalSystem( mov_type = "rhythmic" )
    basis_functions2 = BasisFunctions( mov_type = "rhythmic", n_bfs = n_bfs, cs = cs2 )

    # Creating a subplot    
    fig2, ax2 = plt.subplots( figsize = (8, 4) )

    # Plot the 10 Basis Functions
    for i in range( n_bfs ):
        ax2.plot( cs2.get_value( t_arr ), basis_functions2.calc_ith_activation( i, cs2.get_value( t_arr ) )  )
    
    ax2.set_title( str( n_bfs ) + " Rhythmic Basis Functions" )

    plt.show( )    
