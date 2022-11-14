function [ x, dx, ddx ] = min_jerk_traj( t_arr, x0i, x0f, D, ti )

N   = length( t_arr );
nq  = length( x0i );
x   = zeros( nq, N );
dx  = zeros( nq, N );
ddx = zeros( nq, N );

    for i = 1 : N
        t = t_arr( i );
        
        if t <= ti 
           x(   :, i ) = x0i; 
           dx(  :, i ) =   0; 
           ddx( :, i ) =   0; 
           
        elseif t >= ti && t <= ti + D
           
           x( :, i )   =   x0i + ( x0f - x0i ) * ( 10 * ( ( t - ti )/D )^3 -  15 * ( ( t - ti )/D )^4 +   6 * ( ( t - ti )/D )^5 );
           dx( :, i )  =   1/D * ( x0f - x0i ) * ( 30 * ( ( t - ti )/D )^2 -  60 * ( ( t - ti )/D )^3 +  30 * ( ( t - ti )/D )^4 );
           ddx( :, i ) = 1/D^2 * ( x0f - x0i ) * ( 60 * ( ( t - ti )/D )   - 180 * ( ( t - ti )/D )^2 + 120 * ( ( t - ti )/D )^3 );
           
        else
           x(   :, i ) = x0f; 
           dx(  :, i ) =   0; 
           ddx( :, i ) =   0; 
           
        end
    end


end

