% [Project]        DMP Comparison - New Images for the Revision
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Jan. 8th, 2024
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION

clear; close all; clc; workspace;

% Add the Libraries of 
addpath( 'MATLAB_Library/myUtils', 'MATLAB_Library/myGraphics' )

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );     
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

