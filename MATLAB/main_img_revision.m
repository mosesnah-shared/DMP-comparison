% [Project]        DMP Comparison - New Images for the Revision
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Jan. 8th, 2024
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION

clear; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );     

% Add the Libraries of 
addpath( 'MATLAB_Library/myUtils', 'MATLAB_Library/myGraphics' )

myFigureConfig( 'fontsize',  20, ...
               'LineWidth',  10, ...
           'AxesLineWidth', 1.5, ...     For Grid line, axes line width etc.
              'markerSize',  25    )  
             
% Setting color structure 'c' as global variable          
global c                                                                   
c  = myColor(); 
fs = 40;

%% ==================================================================
%% (--) Goal directed Discrete Movement - Joint Space, Sensitivity Analysis
clear data*; clc; close all

% For Different Stiffness Values
% The results are saved under the following directory 
dir_name = '../results/discrete_joint_tracking_error/for_stiffness/';

% Save the data as raw data
% The array of stiffness values used for the comparison
Kq = [ 30, 150, 1200 ];
Nk = length( Kq );
rawDataK = cell( 1, Nk );

for i = 1 : Nk
    rawDataK{ i } = load( [ dir_name, 'Kq', num2str( Kq(i) ), 'Bq50/ctrl_joint_imp.mat' ] );
end

% Adding the Subplots

% ======================================================================== %
% Plot1: EDA, joint 1, Changing Stiffness
subplot( 2, 2, 1 )
hold on
% Kq = [ 30, 150, 1200 ];
lw_s = { ':', '-', '-.' };  
lw2 = 5;
for i = 1 : Nk
    t_arr = rawDataK{ i }.t_arr;
    q_arr = rawDataK{ i }.q_arr( 1, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end
    plot( t_arr, q_arr, 'linewidth', lw2,  'color', c.orange, 'linestyle', lw_s{i} );
end
plot( t_arr, rawDataK{ 1 }.q0_arr( 1, : ), 'linewidth', 2, 'color', 'k', 'linestyle', '-' );
ylabel( 'Joint1, $q_1(t)$ (rad)', 'fontsize', fs  )
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [0, 1.5], 'fontsize', fs  )
legend( '$\mathbf{K}_{q}=30\mathbf{I}_{2}$', '$\mathbf{K}_{q}=150\mathbf{I}_{2}$', '$\mathbf{K}_{q}=1200\mathbf{I}_{2}$', 'fontsize', 30, 'location', 'southeast' )
title( 'Effect of Stiffness', 'fontsize', fs  )


% ======================================================================== %
% Plot2: EDA, joint 2, Changing Stiffness
subplot( 2, 2, 3 )
hold on
for i = 1 : Nk
    t_arr = rawDataK{ i }.t_arr;
    q_arr = rawDataK{ i }.q_arr( 2, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end    
    plot( t_arr, q_arr, 'linewidth', lw2,  'color', c.orange,'linestyle', lw_s{i}  );
end
plot( t_arr, rawDataK{ 1 }.q0_arr( 2, : ), 'linewidth', 2, 'color', 'k', 'linestyle', '-');
ylabel( 'Joint2, $q_2(t)$ (rad)', 'fontsize', fs  )
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [0, 1.2], 'fontsize', fs  )
xlabel( '$t$ (sec)', 'fontsize', 40 )


% For Different Damping Values
dir_name = '../results/discrete_joint_tracking_error/for_damping/';

% Save the data as raw data
% The array of stiffness values used for the comparison
Bq = [ 10, 50, 150 ];
Nb = length( Bq );
rawDataB = cell( 1, Nb );

for i = 1 : Nb
    rawDataB{ i } = load( [ dir_name, 'Kq150Bq', num2str( Bq(i) ), '/ctrl_joint_imp.mat' ] );
end
fs = 40;


% ======================================================================== %
% Plot3: EDA, joint 1, Changing Damping
subplot( 2, 2, 2 )
hold on
for i = 1 : Nb
    t_arr = rawDataB{ i }.t_arr;
    q_arr = rawDataB{ i }.q_arr( 1, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end      
    plot( t_arr, q_arr, 'linewidth', lw2,  'color', c.orange,'linestyle', lw_s{i}  );
end
plot( t_arr, rawDataB{ 1 }.q0_arr( 1, : ), 'linewidth', 2, 'color', 'k', 'linestyle', '-' );
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [0, 1.5], 'fontsize', fs  )
legend( '$\mathbf{B}_{q}=10\mathbf{I}_{2}$', '$\mathbf{B}_{q}=50\mathbf{I}_{2}$', '$\mathbf{B}_{q}=150\mathbf{I}_{2}$', 'fontsize', 30, 'location', 'southeast' )
title( 'Effect of Damping', 'fontsize', fs  )

% ======================================================================== %
% Plot3: EDA, joint 2, Changing Damping
subplot( 2, 2, 4 )
hold on
for i = 1 : Nb
    t_arr = rawDataB{ i }.t_arr;
    q_arr = rawDataB{ i }.q_arr( 2, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end      
    plot( t_arr, q_arr, 'linewidth', lw2,  'color', c.orange,'linestyle', lw_s{i}  );
end

plot( t_arr, rawDataB{ 1 }.q0_arr( 2, : ), 'linewidth', 2, 'color', 'k', 'linestyle', '-' );
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [0, 1.2], 'fontsize', fs  )
xlabel( '$t$ (sec)', 'fontsize', 40 )

mySaveFig( gcf, 'revision_discrete_joint_space_sensitivity' )


%% ==================================================================
%% (--) Goal directed Discrete Movement - Task Space, Sensitivity Analysis
clear raw*; clc; close all

% For Different Stiffness Values
% The results are saved under the following directory 
dir_name = '../results/discrete_task_tracking_error/for_stiffness/';

% Save the data as raw data
% The array of stiffness values used for the comparison
Kp = [ 20, 60, 300 ];
Nk = length( Kp );
rawDataK = cell( 1, Nk );

for i = 1 : Nk
    rawDataK{ i } = load( [ dir_name, 'Kp', num2str( Kp(i) ), 'Bp20/ctrl_task_imp.mat' ] );
end

% ======================================================================== %
% Plot1: EDA, X-coordinate, Changing Stiffness
subplot( 2, 2, 1 )
hold on
% Kq = [ 30, 150, 1200 ];
lw_s = { ':', '-', '-.' };  
lw2 = 5;
for i = 1 : Nk
    t_arr = rawDataK{ i }.t_arr;
    p_arr = rawDataK{ i }.p_arr( 1, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end
    plot( t_arr, p_arr, 'linewidth', lw2,  'color', c.orange, 'linestyle', lw_s{i} );
end
plot( t_arr, rawDataK{ 1 }.p0_arr( 1, : ), 'linewidth', 3, 'color', 'k', 'linestyle', '--' );
ylabel( '$X$ (m)', 'fontsize', fs  )
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [-0.4, 0.4001], 'fontsize', fs  )
legend( '$\mathbf{K}_{p}=20\mathbf{I}_{2}$', '$\mathbf{K}_{p}=60\mathbf{I}_{2}$', '$\mathbf{K}_{p}=300\mathbf{I}_{2}$', 'fontsize', 30, 'location', 'southeast' )
title( 'Effect of Stiffness', 'fontsize', fs  )


% ======================================================================== %
% Plot2: EDA, Y-coordinate, Changing Stiffness
subplot( 2, 2, 3 )
hold on
for i = 1 : Nk
    t_arr = rawDataK{ i }.t_arr;
    p_arr = rawDataK{ i }.p_arr( 2, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end    
    plot( t_arr, p_arr, 'linewidth', lw2,  'color', c.orange,'linestyle', lw_s{i}  );
end
plot( t_arr, rawDataK{ 1 }.p0_arr( 2, : ), 'linewidth', 3, 'color', 'k', 'linestyle', '--' );
ylabel( '$Y$ (m)', 'fontsize', fs  )
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [0.5, 2], 'fontsize', fs  )
xlabel( '$t$ (sec)', 'fontsize', 40 )


% For Different Damping Values
dir_name = '../results/discrete_joint_tracking_error/for_damping/';

% Save the data as raw data
% The array of stiffness values used for the comparison
Bq = [ 10, 50, 150 ];
Nb = length( Bq );
rawDataB = cell( 1, Nb );

for i = 1 : Nb
    rawDataB{ i } = load( [ dir_name, 'Kq150Bq', num2str( Bq(i) ), '/ctrl_joint_imp.mat' ] );
end
fs = 40;

% For Different Damping Values
dir_name = '../results/discrete_task_tracking_error/for_damping/';

% Save the data as raw data
% The array of stiffness values used for the comparison
Bp = [ 10, 20, 60 ];
Nb = length( Bp );
rawDataB = cell( 1, Nb );

for i = 1 : Nb
    rawDataB{ i } = load( [ dir_name, 'Kp60Bp', num2str( Bp(i) ), '/ctrl_task_imp.mat' ] );
end


% ======================================================================== %
% Plot3: EDA, X-coordinate, Changing Damping
subplot( 2, 2, 2 )
hold on
for i = 1 : Nb
    t_arr = rawDataB{ i }.t_arr;
    p_arr = rawDataB{ i }.p_arr( 1, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end      
    plot( t_arr, p_arr, 'linewidth', lw2,  'color', c.orange,'linestyle', lw_s{i}  );
end
plot( t_arr, rawDataK{ 1 }.p0_arr( 1, : ), 'linewidth', 3, 'color', 'k', 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [-0.4, 0.4001], 'fontsize', fs  )
legend( '$\mathbf{B}_{p}=10\mathbf{I}_{2}$', '$\mathbf{B}_{p}=20\mathbf{I}_{2}$', '$\mathbf{B}_{p}=60\mathbf{I}_{2}$', 'fontsize', 30, 'location', 'southeast' )
title( 'Effect of Damping', 'fontsize', fs  )

% Plot4: EDA, Y-coordinate, Changing Damping
subplot( 2, 2, 4 )
hold on
for i = 1 : Nb
    t_arr = rawDataB{ i }.t_arr;
    p_arr = rawDataB{ i }.p_arr( 2, : );
    if i == 2
        lw2 = 5;
    else
        lw2 = 3;
    end      
    plot( t_arr, p_arr, 'linewidth', lw2,  'color', c.orange,'linestyle', lw_s{i}  );
end
plot( t_arr, rawDataK{ 1 }.p0_arr( 2, : ), 'linewidth', 3, 'color', 'k', 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.5 ], 'ylim', [0.5, 2], 'fontsize', fs  )


mySaveFig( gcf, 'revision_discrete_task_space_sensitivity' )

%% ==================================================================
%% (--) Goal directed Discrete Movement - Task Space, Damped Least Square
clear data*; clc; close all

% For Different Stiffness Values
% The results are saved under the following directory 
file_name = '../results/discrete_move_task_space_wo_redund/movement/vertical_stretch_damped_least/ctrl_task_dmp.mat';

data_move = load( file_name );

% ======================================================================== %
% Plot1: Dynamic Movement Primitives
f = figure(); a = axes( 'parent', f );
hold on

g_start = data_move.p_command(  :, 1 );
g_end = data_move.p_command(  :, end );

% Get the x, y position of the joints 
q_abs = cumsum( data_move.q_arr , 1 );
x_arr = cumsum( cos( q_abs ), 1 );
y_arr = cumsum( sin( q_abs ), 1 );


alpha_arr = [0.2, 0.3, 0.3, 0.7, 1.0];
idx_arr   = [1, 400, 600, 800, 1100];
for i = 1 : length( idx_arr )
    idx = idx_arr( i );
    alpha = alpha_arr( i );
    scatter( [ 0, x_arr( 1:end-1, idx  )' ] , [ 0, y_arr(1:end-1, idx )' ], 400, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    p2 = plot( [ 0, x_arr( :, idx )' ] , [ 0, y_arr( :, idx)' ], 'color', c.black, 'linewidth', 4 );
    p2.Color( 4 ) = alpha;
    scatter( x_arr( end, idx ), y_arr( end, idx  ),  1200,  'markerfacecolor', c.blue, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

end

plot( data_move.p_arr( 1, 1:1000 ), data_move.p_arr( 2, 1:1000  ) , 'linewidth', 8, 'color', c.blue )
plot( data_move.p_command(  1, : ), data_move.p_command( 2, : ), 'linewidth', 4, 'color', c.black, 'linestyle',  '--' )

% Start and End Location
scatter( 0, g_start( 2 ), 300, 'o', 'markerfacecolor', c.pink_sunset, 'markeredgecolor', c.black, 'markerfacealpha', 1.0 )
scatter( 0,   g_end( 2 ), 300, 'square', 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'markerfacealpha', 1.0 )

text( -0.75, g_start( 2 ), 'Start $\mathbf{p}_i$' , 'fontsize', fs)
text( -0.75, g_end( 2 ), 'Goal $\mathbf{g}$'   , 'fontsize', fs )


xlabel( '$X$ (m)', 'fontsize', fs );
ylabel( '$Y$ (m)', 'fontsize', fs )
axis equal

set( gca, 'xlim', [-1.1, 1.1] , 'ylim', [-0.2, 2.4], 'xtick', [-1.0, 0.0, 1.0], 'ytick', [0.0, 1.0, 2.0], 'fontsize', 1.2*fs ) 
% title( 'Dynamic Movement Primitives', 'fontsize', fs )

mySaveFig( gcf, 'revision_discrete_task_space_DLS' )

%% =======================================================================
%% (--) Goal directed Discrete Movement - Position and Orientation, EDA #1

close all;

% For visualization, we will use Explicit
file_name = '../results/position_and_orientation/motor/data.mat';

data_move = load( file_name );

% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data_move.t_arr );
time_arr = [ 1, 1300, 1900, Np ];

f = figure( ); a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );


for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8500 0.3250 0.0980], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data_move.p_links( step , i, : ) ); 
        R_tmp = squeeze( data_move.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data_move.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data_move.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
    
end

plot3( a, data_move.p0_arr( :, 1 ), data_move.p0_arr( :, 2 ), data_move.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
plot3( a, data_move.p_arr(  :, 1 ), data_move.p_arr(  :, 2 ), data_move.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-', 'color', [0.8500 0.3250 0.0980] )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'visible', 'off', 'xlim', [-0.2794,0.9169], 'ylim', [-0.4136,0.5], 'zlim', [-0.0270,0.6121] )

mySaveFig( gcf, 'revision_EDA1_img1' )


%% (--) Goal directed Discrete Movement - Position and Orientation, EDA #2

% Drawing the position and orientation error 
a1 = subplot( 2, 1, 1 );
hold on

t_arr  = data_move.t_arr;
p_arr  = data_move.p_arr;
p0_arr = data_move.p0_arr;
plot( a1, t_arr, p_arr( :, 1 )-p0_arr( :, 1 ), 'linewidth', 5 )
plot( a1, t_arr, p_arr( :, 2 )-p0_arr( :, 2 ), 'linewidth', 5 )
plot( a1, t_arr, p_arr( :, 3 )-p0_arr( :, 3 ), 'linewidth', 5 )

% Calculate the Geodesic distaince
Np   = length( t_arr );
dist = zeros( 1, Np );
for i = 1 : Np
    R1 = squeeze( data_move.R_arr( i, :, : ) ); 
    R2 = squeeze( data_move.R0_arr( i, :, : ) );
    dist( i ) = geodesicDistance( R1, R2 );
end

a2 = subplot( 2, 1, 2 );
plot( t_arr, dist, 'linewidth', 8, 'color', 'k' )
hold on

mySaveFig( gcf, 'revision_EDA1_img2' )


%% (--) Goal directed Discrete Movement - Position and Orientation, EDA #3
close all;
f = figure( ); a = axes( 'parent', f );
hold on
plot3( a, data_move.p0_arr( :, 1 ), data_move.p0_arr( :, 2 ), data_move.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
plot3( a, data_move.p_arr(  :, 1 ), data_move.p_arr(  :, 2 ), data_move.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-', 'color', [0.8500 0.3250 0.0980] )
axis equal

set( a, 'visible', 'off' )
view( [90, 0 ])

Np = length( data_move.t_arr );

time_arr = [ 1, 1000, 1400, 1800, 2200, Np ];

for i = 1: length( time_arr )
    
    step = time_arr( i );
    
    % Adding the markers and also orientation
    p_tmp = data_move.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data_move.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
    R_tmp = squeeze( data_move.R0_arr( step , :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
        
    
end
set( a, 'xlim', [0.3251, 1.6981], 'ylim', [-0.2401, 0.29], 'zlim', [0.1633, 0.3969] )
mySaveFig( gcf, 'revision_EDA1_img3' )


%% =======================================================================
%% (--) Goal directed Discrete Movement - Position and Orientation, DMP #1
close all;
% For visualization, we will use Explicit
file_name = '../results/position_and_orientation/movement/data.mat';
data_move = load( file_name );

% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data_move.t_arr );
time_arr = [ 1, 1200, 1800, Np ];


f = figure( ); a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );


for j = 1 : length( time_arr )
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.3010 0.7450 0.9330]	, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.5 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data_move.p_links( step , i, : ) ); 
        R_tmp = squeeze( data_move.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data_move.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data_move.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', [0 0.4470 0.7410]	, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
    
end

plot3( a, data_move.p0_arr( :, 1 ), data_move.p0_arr( :, 2 ), data_move.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
plot3( a, data_move.p_arr(  :, 1 ), data_move.p_arr(  :, 2 ), data_move.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-', 'color', [0 0.4470 0.7410]	 )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'visible', 'off', 'xlim', [-0.2794,0.9169], 'ylim', [-0.4136,0.5], 'zlim', [-0.0270,0.6121] )

mySaveFig( gcf, 'revision_DMP_img1' )

%% (--) Goal directed Discrete Movement - Position and Orientation, DMP #2

% Drawing the position and orientation error 
a1 = subplot( 2, 1, 1 );
hold on

t_arr  = data_move.t_arr;
p_arr  = data_move.p_arr;
p0_arr = data_move.p0_arr;
plot( a1, t_arr, p_arr( :, 1 )-p0_arr( :, 1 ), 'linewidth', 5 )
plot( a1, t_arr, p_arr( :, 2 )-p0_arr( :, 2 ), 'linewidth', 5 )
plot( a1, t_arr, p_arr( :, 3 )-p0_arr( :, 3 ), 'linewidth', 5 )

% Calculate the Geodesic distaince
Np   = length( t_arr );
dist = zeros( 1, Np );
for i = 1 : Np
    R1 = squeeze( data_move.R_arr( i, :, : ) ); 
    R2 = squeeze( data_move.R0_arr( i, :, : ) );
    dist( i ) = geodesicDistance( R1, R2 );
end

a2 = subplot( 2, 1, 2 );
plot( t_arr, dist, 'linewidth', 8, 'color', 'k' )
hold on

mySaveFig( gcf, 'revision_DMP_img2' )

%% (--) Goal directed Discrete Movement - Position and Orientation, DMP #3
close all;
f = figure( ); a = axes( 'parent', f );
hold on
plot3( a, 0.1+data_move.p0_arr( :, 1 ), data_move.p0_arr( :, 2 ), data_move.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
plot3( a, data_move.p_arr(  :, 1 ), data_move.p_arr(  :, 2 ), data_move.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-', 'color', [0 0.4470 0.7410]	 )
axis equal

set( a, 'visible', 'off' )
view( [90, 0 ])

Np = length( data_move.t_arr );

time_arr = [ 1, 1000, 1400, 1700, 2100, 2800 ];

for i = 1: length( time_arr )
    
    step = time_arr( i );
    
    % Adding the markers and also orientation
    p_tmp = data_move.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data_move.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
    R_tmp = squeeze( data_move.R0_arr( step , :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
        
    
end

set( a, 'xlim', [0.3251, 1.0981], 'ylim', [-0.2401, 0.2680], 'zlim', [0.1633, 0.3969] )

    
mySaveFig( gcf, 'revision_DMP_img3' )
