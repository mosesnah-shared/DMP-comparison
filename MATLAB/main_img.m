% [Project]        DMP Comparison - Video Generation
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Oct. 23th, 2022
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
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor(); 

global mode 
mode = "MOVEMENT"; % Either movement or motor or both to generate the images 

%% ==================================================================
%% (--) Goal directed Discrete Movement - Joint Space

file_name1 = '../results/discrete_move_joint_space/motor/ctrl_joint_imp.mat';


% Motor Primitives
data_raw1 = load( file_name1 );
data_raw1.q0i = double( data_raw1.q0i );
data_raw1.q0f = double( data_raw1.q0f );
data_raw1.D   = double( data_raw1.D   );
data_raw1.ti  = double( data_raw1.ti  );


[ q0_arr, dq0_arr, ddq0_arr ] = min_jerk_traj( data_raw1.t_arr, data_raw1.q0i', data_raw1.q0f', data_raw1.D, data_raw1.ti );


% Movement Primitives
file_name2 = '../results/discrete_move_joint_space/movement/dmp1.mat';
file_name3 = '../results/discrete_move_joint_space/movement/dmp2.mat';

data_raw_q1 = load( file_name2 );
data_raw_q2 = load( file_name3 );

subplot( 2, 2, 1 )

plot( data_raw1.t_arr, q0_arr( 1, : ), 'color', c.pink, 'linewidth', 4, 'linestyle', '--' );
hold on
N = length( data_raw_q1.q_result1 );
dt = data_raw_q1.dt;
plot( dt * ( 0 : ( N - 1) ), data_raw_q1.q_result1, 'linewidth', 4,  'color', 'k' );
set( gca, 'xlim', [ 1, 4 ], 'ylim', [0, 1.1], 'xtick', [ 1, 2, 3, 4 ], 'xticklabel', { '0', '1', '2', '3' },'fontsize', 30 )
ylabel( 'Joint1, $q_1$', 'fontsize', 30 )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )
% yline( 1.0, 'linewidth', 3, 'linestyle', '-.')
% legend( '$q(t)$', '$q_{des}(t)$', 'location', 'northwest' )


subplot( 2, 2, 2 )

plot( data_raw1.t_arr, q0_arr( 1, : ), 'color', c.pink, 'linewidth', 4, 'linestyle', '--' );
hold on
plot( data_raw1.t_arr, data_raw1.q_arr( :, 1 ), 'linewidth', 4,  'color', 'k' );
set( gca, 'xlim', [ 1, 4 ], 'ylim', [0, 1.1], 'xtick', [ 1, 2, 3, 4 ], 'xticklabel', { '0', '1', '2', '3' },'fontsize', 30 )
title( 'Dynamic Motor Primitives', 'fontsize', 30 )
% yline( 1.0, 'linewidth', 3, 'linestyle', '-.')


subplot( 2, 2, 3 )
ylabel( 'Joint2, $q_1$', 'fontsize', 30 )
plot( data_raw1.t_arr, q0_arr( 2, : ), 'color', c.pink, 'linewidth', 4, 'linestyle', '--' );
hold on
N = length( data_raw_q1.q_result1 );
dt = data_raw_q1.dt;
plot( dt * ( 0 : ( N - 1) ), data_raw_q2.q_result2, 'linewidth', 4,  'color', 'k' );
set( gca, 'xlim', [ 1, 4 ], 'ylim', [0, 1.1], 'xtick', [ 1, 2, 3, 4 ], 'xticklabel', { '0', '1', '2', '3' },'fontsize', 30 )
xlabel( '$t$', 'fontsize', 30 )
ylabel( 'Joint2, $q_2$', 'fontsize', 30 )
% yline( 1.0, 'linewidth', 3, 'linestyle', '-.')

subplot( 2, 2, 4 )
plot( data_raw1.t_arr, q0_arr( 2, : ), 'color', c.pink, 'linewidth', 4, 'linestyle', '--' );
hold on
plot( data_raw1.t_arr, data_raw1.q_arr( :, 2 ), 'linewidth', 4,  'color', 'k' );
set( gca, 'xlim', [ 1, 4 ], 'ylim', [0, 1.1], 'xtick', [ 1, 2, 3, 4 ], 'xticklabel', { '0', '1', '2', '3' },'fontsize', 30 )
xlabel( '$t$', 'fontsize', 30 )
% yline( 1.0, 'linewidth', 3, 'linestyle', '-.')


mySaveFig( gcf, 'goal_directed_discrete_joint_space' )

%% ==================================================================
%% (--) Goal directed Discrete Movement - Task-Space

% The number of targets
N = 8;
data_raw1 = cell( 8 ); 

c_arr = [ c.blue; c.orange; c.green; c.purple; c.roseRed; c.peach; c.pink; c.blue_sky ];

for i = 1 : 8
    file_name = ['../results/discrete_move_task_space_wo_redund/motor/target', num2str( i ), '.mat' ];
    data_raw1{ i } = load( file_name );
end

subplot( 1, 2, 1)
hold on

xEEi = data_raw1{ i }.xEE_arr( 1, : );
r = 0.5;
for i = 1 : 8
    plot( r * cos( ( i - 1 ) * pi/4 ) , r * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 30, 'markerfacecolor', c_arr( i, : ), 'markeredgecolor', c_arr( i, : ), 'linewidth', 3 )
    plot( data_raw1{ i }.xEE_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.xEE_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 8, 'color', c_arr( i, : ) )
    plot( data_raw1{ i }.x0_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.x0_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
end

xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
title( 'Dynamic Motor Primitives', 'fontsize', 30 )
axis equal
set( gca, 'xlim', [-0.6, 0.600001], 'ylim', [-0.6, 0.600001] )

subplot( 1, 2, 2)
hold on

data_raw2 = cell( 8 ); 

for i = 1 : 8
    file_name = ['../results/discrete_move_task_space_wo_redund/movement/target', num2str( i ), '.mat' ];
    data_raw2{ i } = load( file_name );
end

for i = 1 : 8
    plot( r * cos( ( i - 1 ) * pi/4 ) , r * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 30, 'markerfacecolor', c_arr( i, : ), 'markeredgecolor', c_arr( i, : ), 'linewidth', 3 )
    plot( data_raw2{ i }.x - xEEi( 1 ), data_raw2{ i }.y - xEEi( 2 ), 'linewidth', 8, 'color', c_arr( i, : ) )
    plot( data_raw1{ i }.x0_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.x0_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
end

xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )
axis equal
set( gca, 'xlim', [-0.6, 0.600001], 'ylim', [-0.6, 0.600001] )

mySaveFig( gcf, 'goal_directed_discrete_task_space' )


%% ==================================================================
%% (--) Goal directed Discrete Movement - With Redundancy #1

% Dynamic Movement Primitives
file_name1 = '../results/discrete_move_task_space_w_redund/movement/dmp.mat';
data_raw1 = load( file_name1 );

dt = data_raw1.dt;
N  = length( data_raw1.x );

t_arr = dt * ( 1 : N ) - dt;


% Dynamic Motor Primitives
file_name2 = '../results/discrete_move_task_space_w_redund/motor/ctrl_joint_imp.mat';
file_name3 = '../results/discrete_move_task_space_w_redund/motor/ctrl_task_imp.mat';
data_raw2 = load( file_name2 );
data_raw3 = load( file_name3 );

% plot( t_arr, data_raw1.dq )
subplot( 2, 2, [1,2])
hold on


plot( data_raw1.x, data_raw1.y, 'linewidth', 4, 'linestyle', '--', 'color', c.black )
if mode == "MOVEMENT" || mode == "BOTH"
    plot( data_raw1.p( :, 1), data_raw1.p( :, 2  ), 'linewidth', 6, 'color', c.blue )
elseif mode == "MOTOR" || mode == "BOTH"
    plot( data_raw3.x0_arr( : , 1 ), data_raw3.x0_arr( : , 2 ), 'linewidth', 6, 'color', c.orange )
end

set( gca, 'xlim', [-.2, 3.2], 'xtick', [ 0, 1.5, 3.0 ], 'ylim', [ 2.5, 3.5], 'ytick', [ 2.5, 3.0, 3.5 ], 'fontsize', 30 )
text( data_raw3.x0_arr( 1 , 1 ), data_raw3.x0_arr( 1 , 2 )- 0.1, 'Start $\mathbf{p}_i$' )
text( data_raw3.x0_arr( end , 1 )-0.2, data_raw3.x0_arr( end , 2 )- 0.1, 'Goal $\mathbf{g}$' )
scatter( data_raw3.x0_arr( end , 1 ), data_raw3.x0_arr( end , 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )
scatter( data_raw3.x0_arr( 1 , 1 ), data_raw3.x0_arr( 1 , 2 ), 300, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )

if mode == "MOVEMENT" 
    legend( '', 'Dynamic Movement Primitives', 'location', 'northwest', 'fontsize', 30  )
elseif mode == "MOTOR"
    legend( '', 'Dynamic Motor Primitives', 'location', 'northwest', 'fontsize', 30  )
else
    legend( '', 'Dynamic Movement Primitives', 'Dynamic Motor Primitives', 'location', 'northwest', 'fontsize', 30  )
end


if mode == "MOVEMENT" || mode == "BOTH"
    subplot( 2, 2, 3)
    hold on
    plot( data_raw1.x, data_raw1.y, 'linewidth', 2, 'linestyle', '--', 'color', c.black )

    % Get the x, y position of the joints 
    q_abs = cumsum( data_raw1.q , 2 );
    x_arr = cumsum( cos( q_abs ), 2 );
    y_arr = cumsum( sin( q_abs ), 2 );

    alpha_arr = [0.2, 0.5, 1.0];
    idx_arr   = [1000, 2300, 5000];
    for i = 1 : 3
        idx = idx_arr( i );
        alpha = alpha_arr( i );
        scatter( [ 0, x_arr( idx, 1:end-1 ) ] , [ 0, y_arr( idx, 1:end-1) ], 200, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
        p2 = plot( [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, :) ], 'color', c.black, 'linewidth', 4 );
        p2.Color( 4 ) = alpha;
        scatter( x_arr( idx, end ), y_arr( idx, end ),  600,  'markerfacecolor', c.blue, 'markeredgecolor', c.blue, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

    end

    set( gca, 'xlim', [ -1.0, 4.0] , 'ylim', [-1.0, 4.0], 'xticklabel', {}, 'yticklabel', {} )
    axis equal
    title( 'Dynamic Movement Primitives', 'fontsize', 30 )
end

if mode == "MOTOR" || mode == "BOTH"
    subplot( 2, 2, 4)
    hold on
    plot( data_raw1.x, data_raw1.y, 'linewidth', 2, 'linestyle', '--', 'color', c.black )

    % Get the x, y position of the joints 
    q_abs = cumsum( data_raw2.q_arr , 2 );
    x_arr = cumsum( cos( q_abs ), 2 );
    y_arr = cumsum( sin( q_abs ), 2 );

    alpha_arr = [0.2, 0.5, 1.0];
    idx_arr   = [30, 100, 300];
    for i = 1 : 3
        idx = idx_arr( i );
        alpha = alpha_arr( i );
        scatter( [ 0, x_arr( idx, 1:end-1 ) ] , [ 0, y_arr( idx, 1:end-1) ], 200, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
        p2 = plot( [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, :) ], 'color', c.black, 'linewidth', 4 );
        p2.Color( 4 ) = alpha;
        scatter( x_arr( idx, end ), y_arr( idx, end ),  600,  'markerfacecolor', c.orange, 'markeredgecolor', c.orange, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

    end

    title( 'Dynamic Motor Primitives', 'fontsize', 30 )
    set( gca, 'xlim', [ -1.0, 4.0] , 'ylim', [-1.0, 4.0], 'xticklabel', {}, 'yticklabel', {} )
    axis equal
end

mySaveFig( gcf, 'redundancy_end_effector' )


%% (--) Goal directed Discrete Movement - With Redundancy #2
figure( )

% Plot the qdot and q
subplot( 2, 2, 1)
hold on
plot( t_arr, data_raw1.q )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )
ylabel( '$\mathbf{q}(t)$' )
set( gca, 'fontsize', 30, 'xlim', [1, 5 ], 'ylim', [ -1, 2], 'xticklabel', {}  )

subplot( 2, 2, 2)
hold on
plot( data_raw2.t_arr, data_raw2.q_arr )
set( gca, 'fontsize', 30, 'xlim', [1, 5 ], 'ylim', [ -1, 2], 'xticklabel', {}  )
title( 'Dynamic Motor Primitives', 'fontsize', 30 )

subplot( 2, 2, 3)
hold on
plot( t_arr, data_raw1.dq )
xlabel( '$t$' )
ylabel( '$\dot{\mathbf{q}}(t)$' )
set( gca, 'fontsize', 30, 'xlim', [1, 5 ], 'ylim', [ -1, 0.5], 'xticklabel', { '0', '1', '2', '3', '4' }  )

subplot( 2, 2, 4)
hold on
plot( data_raw2.t_arr, data_raw2.dq_arr )
xlabel( '$t$' )
set( gca, 'fontsize', 30, 'xlim', [1, 5 ], 'ylim', [ -1, 0.5],  'xticklabel', { '0', '1', '2', '3', '4' }  )
legend( '$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', 'location', 'southeast')
mySaveFig( gcf, 'redundancy_joint_trajs' )

%% ==================================================================
%% (--) Sequence of Discrete Movements #1

file_name1 = '../results/sequence/movement/dmp.mat';
file_name2 = '../results/sequence/motor/dmp.mat';

data_raw1 = load( file_name1 );
data_raw2 = load( file_name2 );

% Get the initial end-effector position 
g_old = data_raw2.p0_arr( 1, : ) + [ -0.7, 0.7, 0. ];
g_new = g_old + [ 1.5, 0.5, 0. ];

 
subplot( 2, 2, [1,3] )
hold on
if mode == "MOVEMENT" || mode == "BOTH"
    plot( data_raw1.p( 1, : ), data_raw1.p( 2, : ), 'linewidth', 4, 'color', c.blue)
elseif mode == "MOTOR" || mode == "BOTH"
    plot( data_raw2.p_arr( :, 1 ), data_raw2.p_arr( :, 2 ), 'linewidth', 4, 'color', c.orange  )
end
scatter( g_old( 1 ), g_old( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.3 )
scatter( g_new( 1 ), g_new( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )
scatter( data_raw2.p0_arr( 1, 1 ), data_raw2.p0_arr( 1, 2 ), 300, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
xlabel( 'X (m)', 'fontsize', 35 )
ylabel( 'Y (m)', 'fontsize', 35 )
text( g_old( 1 ) - 0.1, g_old( 2 ) + 0.1, '$\mathbf{g}_{old}$' );
text( g_new( 1 ) - 0.1, g_new( 2 ) - 0.1, '$\mathbf{g}_{new}$' );
text( data_raw2.p0_arr( 1, 1 ) + 0.1, data_raw2.p0_arr( 1, 2 ), '$\mathbf{p}_{i}$' );

if mode == "MOVEMENT" 
    legend( 'Dynamic Movement Primitives', 'location', 'northwest', 'fontsize', 23)
elseif mode == "MOTOR"
    legend( 'Dynamic Motor Primitives', 'location', 'northwest', 'fontsize', 23  )
else
    legend( 'Dynamic Movement Primitives', 'Dynamic Motor Primitives', 'location', 'northwest', 'fontsize', 23  )
end


set( gca, 'fontsize', 30 )

if mode == "MOVEMENT" || mode == "BOTH"
    subplot( 2, 2, 2 )
    hold on

    % Get the x, y position of the joints 
    q_abs = cumsum( data_raw1.q , 2 );
    x_arr = cumsum( cos( q_abs ), 2 );
    y_arr = cumsum( sin( q_abs ), 2 );

    alpha_arr = [0.2, 0.4, 0.8, 1.0];
    idx_arr   = [1000, 1900, 3000, 7000];
    for i = 1 : length( idx_arr )
        idx = idx_arr( i );
        alpha = alpha_arr( i );
        scatter( [ 0, x_arr( idx, 1:end-1 ) ] , [ 0, y_arr( idx, 1:end-1) ], 200, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
        p2 = plot( [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, :) ], 'color', c.black, 'linewidth', 4 );
        p2.Color( 4 ) = alpha;
        scatter( x_arr( idx, end ), y_arr( idx, end ),  600,  'markerfacecolor', c.blue, 'markeredgecolor', c.blue, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

    end
    plot( x_arr( :, 2) , y_arr( :, 2), 'color',c.blue, 'linewidth', 4 ) 
    scatter( g_old( 1 ), g_old( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.3 )
    scatter( g_new( 1 ), g_new( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )

    text( g_old( 1 ) - 0.1, g_old( 2 ) + 0.3, '$\mathbf{g}_{old}$' );
    text( g_new( 1 ) + 0.1, g_new( 2 ) - 0.1, '$\mathbf{g}_{new}$' );

    title( 'Dynamic Movement Primitives', 'fontsize', 30 )
    xlabel( 'X (m)', 'fontsize', 35 )
    ylabel( 'Y (m)', 'fontsize', 35 )
    set( gca, 'xticklabel', {}, 'yticklabel', {} ,'xlim', [ -1.0, 1.0], 'ylim', [-0.1, 1.9 ] )
    axis equal
end

if mode == "MOTOR" || mode == "BOTH"
    subplot( 2, 2, 4 )
    hold on
    q_abs = cumsum( data_raw2.q_arr , 2 );
    x_arr2 = cumsum( cos( q_abs ), 2 );
    y_arr2 = cumsum( sin( q_abs ), 2 );

    alpha_arr = [0.2, 0.4, 0.8, 1.0];
    idx_arr   = [1000, 1800, 2000, 5000];
    for i = 1 : length( idx_arr )
        idx = idx_arr( i );
        alpha = alpha_arr( i );
        scatter( [ 0, x_arr2( idx, 1:end-1 ) ] , [ 0, y_arr2( idx, 1:end-1) ], 200, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
        p2 = plot( [ 0, x_arr2( idx, : ) ] , [ 0, y_arr2( idx, :) ], 'color', c.black, 'linewidth', 4 );
        p2.Color( 4 ) = alpha;
        scatter( x_arr2( idx, end ), y_arr2( idx, end ),  600,  'markerfacecolor', c.orange, 'markeredgecolor', c.blue, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

    end
    plot( x_arr2( :, 2) , y_arr2( :, 2), 'color',c.orange, 'linewidth', 4 ) 
    scatter( g_old( 1 ), g_old( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.3 )
    scatter( g_new( 1 ), g_new( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )

    text( g_old( 1 ) - 0.1, g_old( 2 ) + 0.3, '$\mathbf{g}_{old}$' );
    text( g_new( 1 ) + 0.1, g_new( 2 ) - 0.1, '$\mathbf{g}_{new}$' );

    set( gca, 'xticklabel', {}, 'yticklabel', {},'xlim', [ -1.0, 1.0], 'ylim', [-0.1, 1.9 ] )
    axis equal
    title( 'Dynamic Motor Primitives', 'fontsize', 30 )
    xlabel( 'X (m)', 'fontsize', 35 )
    ylabel( 'Y (m)', 'fontsize', 35 )
end

mySaveFig( gcf, 'sequence_figure1' )

%% (--) Sequence of Discrete Movements #2
figure( )
subplot( 2, 1, 1 )
hold on
% xlabel( '$t$', 'fontsize', 35 )
ylabel( '$p_x(t)$', 'fontsize', 35 )

if mode == "MOVEMENT" || mode == "BOTH"
    plot( data_raw1.t_arr- 1, data_raw1.p( 1, : ), 'linewidth', 4, 'color', c.blue )
elseif mode == "MOTOR" || mode == "BOTH"
    plot( data_raw2.t_arr -1, data_raw2.p_arr( :, 1 ), 'linewidth', 4, 'color', c.orange )
end    
xline( 0.5, 'linewidth', 2, 'linestyle', '-.' )
yline( g_new( 1 ), 'linewidth', 2, 'linestyle', '-.' )
set( gca, 'fontsize', 30, 'xlim', [0, 4], 'xtick', [ 0, 0.5, 1, 2, 3, 4 ], 'xticklabel', { '0', '$\mathbf{g}_{new}$ Appear', '', 2, '', 4}, ...
                                  'ytick', [ -0.5, 0.0, 0.5, g_new(1), 1.0 ], 'yticklabel', { '-0.5', '0.0', '0.5', '$g_{new,x}$', '' } )
                              

if mode == "MOVEMENT" 
    legend( 'Dynamic Movement Primitives', 'location', 'southeast', 'fontsize', 30)
elseif mode == "MOTOR"
    legend( 'Dynamic Motor Primitives', 'location', 'southeast', 'fontsize', 30  )
else
    legend( 'Dynamic Movement Primitives', 'Dynamic Motor Primitives', 'location', 'southeast', 'fontsize', 30 )
end

subplot( 2, 1, 2 )
hold on
xlabel( '$t$', 'fontsize', 35 )
ylabel( '$p_y(t)$', 'fontsize', 35 )
if mode == "MOVEMENT" || mode == "BOTH"
    plot( data_raw1.t_arr - 1, data_raw1.p( 2, : ), 'linewidth', 4, 'color', c.blue )
elseif mode == "MOTOR" || mode == "BOTH"    
    plot( data_raw2.t_arr - 1, data_raw2.p_arr( :, 2 ), 'linewidth', 4, 'color', c.orange )
end    
xline( 0.5, 'linewidth', 2, 'linestyle', '-.' )
yline( g_new( 2 ), 'linewidth', 2, 'linestyle', '-.' )
set( gca, 'fontsize', 30, 'xlim', [0, 4], 'xtick', [ 0, 0.5, 1, 2, 3, 4 ], 'xticklabel', { '0', '$\mathbf{g}_{new}$ Appear', '', 2, '', 4}, ...
                                          'ytick', [ 0.5, 1.0, 1.5, g_new(2), 2.0 ], 'yticklabel', { '0.5', '1.0', '1.5', '$g_{new,y}$', '' })

if mode == "MOVEMENT" 
    legend( 'Dynamic Movement Primitives', 'location', 'southeast', 'fontsize', 30)
elseif mode == "MOTOR"
    legend( 'Dynamic Motor Primitives', 'location', 'southeast', 'fontsize', 30 )
else
    legend( 'Dynamic Movement Primitives', 'Dynamic Motor Primitives', 'location', 'southeast', 'fontsize',30 )
end
mySaveFig( gcf, 'sequence_figure2' )



%% ==================================================================
%% (--) Task-space Trajectory Tracking --- Rhythmic
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/task_space_traj_track/dynamic_motor/rhythmic.mat';
data_raw1 = load( file_name );

subplot( 1, 2, 2)
hold on

centers = data_raw1.centers;

plot( data_raw1.xEE_arr( :, 1 )-centers(1), data_raw1.xEE_arr( :, 2 )- centers(2), 'linewidth', 6, 'color', c.orange )
plot( data_raw1.x0_arr( :, 1 )-centers(1), data_raw1.x0_arr( :, 2 )- centers(2), 'linewidth', 3, 'linestyle', '--', 'color', 'k')
 
xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
axis equal
set( gca, 'xlim', [-0.6, 0.60001], 'ylim', [-0.6, 0.60001], 'fontsize', 30 )
title( 'Dynamic Motor Primitives', 'fontsize', 30 )


file_name = '../results/task_space_traj_track/dynamic_movement/rhythmic.mat';
data_raw2 = load( file_name );

subplot( 1, 2, 1 )
hold on


plot( data_raw2.y_arr1-centers(1), data_raw2.y_arr2-centers(2), 'linewidth', 6, 'color', c.blue )
plot( data_raw2.y_des1-centers(1), data_raw2.y_des2-centers(2), 'linewidth', 3, 'linestyle', '--', 'color', 'k')
 
xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
axis equal
set( gca, 'xlim', [-0.6, 0.60001], 'ylim', [-0.6, 0.60001], 'fontsize', 30 )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )

mySaveFig( gcf,  'task_space_rhythmic' )

%% ==================================================================
%% (--) Discrete and Rhythmic Movements 

% Dynamic Movement Primitives
% Dynamic Motor Primitives
file_name1 = '../results/discrete_and_rhythmic/movement/dmp.mat';
data_raw1 = load( file_name1 );
subplot( 2, 1, 1 )
hold on
plot( data_raw1.t_arr, data_raw1.g, 'linewidth', 2, 'linestyle', '-', 'color', c.black )
plot( data_raw1.t_arr, data_raw1.g0, 'linewidth', 2, 'linestyle', '-.', 'color', c.black )
plot( data_raw1.t_arr, data_raw1.q( :, 2 ), 'linewidth', 6, 'color', c.blue )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )
ylabel( '$q_{elbow}(t)$', 'fontsize', 30 )
legend( '$g$', '$g_0$' )
set( gca, 'xlim', [0, 17.4], 'fontsize', 30  )

% Dynamic Motor Primitives
file_name2 = '../results/discrete_and_rhythmic/motor/ctrl_joint_imp.mat';
data_raw2 = load( file_name2 );
subplot( 2, 1, 2 )
hold on
plot( data_raw2.t_arr, data_raw2.q0_tmp_arr( :, 2), 'linewidth', 2, 'color', c.black )
plot( data_raw2.t_arr, data_raw2.q_arr( :, 2 ), 'linewidth', 4, 'color', c.orange )
set( gca, 'xlim', [0, 17.4 ], 'fontsize', 30  )
xlabel( '$t$', 'fontsize', 30 )
ylabel( '$q_{elbow}(t)$', 'fontsize', 30 )
title( 'Dynamic Motor Primitives', 'fontsize', 30 )

mySaveFig( gcf,  'discrete_and_rhythmic' )


%% ==================================================================
%% (--) Obstacle Avoidance 
%% -- (-A) Dynamic Motor Primitives - Order 


% Dynamic Movement Primitives
file_name1 = '../results/obstacle_avoidance/movement/dmp.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/obstacle_avoidance/motor/ctrl_task_imp.mat';
file_name3 = '../results/obstacle_avoidance/motor/ctrl_task_imp2.mat';
data_raw2 = load( file_name2 );
data_raw3 = load( file_name3 );

subplot( 2, 2, [1,3] )
hold on
if mode == "MOVEMENT" || mode == "BOTH"
    plot( data_raw1.p( 1, : ), data_raw1.p( 2, : ), 'linewidth', 4, 'color', c.blue )
elseif mode == "MOTOR" || mode == "BOTH"    
    plot( data_raw2.xEE_arr( :, 1 ), data_raw2.xEE_arr( :, 2 ), 'linewidth', 4, 'color', c.orange )
end
plot( data_raw2.x0_arr( :, 1), data_raw2.x0_arr( :, 2),   'linewidth', 4, 'linestyle', '-.', 'color', c.black )
scatter( data_raw2.x0i( 1 ), data_raw2.x0i( 2 ), 200, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
scatter( data_raw2.x0f( 1 ), data_raw2.x0f( 2 ), 200, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
scatter( 0.5 *( data_raw2.x0i( 1 ) + data_raw2.x0f( 1 ) ), 0.5 *( data_raw2.x0i( 2 ) + data_raw2.x0f( 2 ) ) ...
        , 1000, 'o', 'markerfacecolor', c.grey, 'markeredgecolor', c.black, 'linewidth', 3 )
set( gca, 'xlim', [-.4, .40001], 'ylim', [ 0.3, 1.8],  'fontsize', 30 )
text( -0.25 + 0.5 *( data_raw2.x0i( 1 ) + data_raw2.x0f( 1 ) ), 0.5 *( data_raw2.x0i( 2 ) + data_raw2.x0f( 2 ) ), 'Obstacle' )
text( data_raw2.x0i( 1 )-0.08, data_raw2.x0i( 2 ), '$\mathbf{p}_i$' )
text( data_raw2.x0f( 1 )-0.08, data_raw2.x0f( 2 ), '$\mathbf{g}$' )
% daspect([1 1 1])
if mode == "MOVEMENT"
    legend( 'Dynamic Movement Primitives', 'fontsize', 30 , 'location', 'southwest' )
elseif mode == "MOTOR"
    legend( 'Dynamic Motor Primitives', 'fontsize', 30 , 'location', 'southwest' )
else
    legend( 'Dynamic Movement Primitives', 'Dynamic Motor Primitives', 'fontsize', 30 , 'location', 'southwest' )
end    
xlabel( 'X (m)', 'fontsize', 35 )
ylabel( 'Y (m)', 'fontsize', 35 )


if mode == "MOVEMENT" || mode == "BOTH" 
    subplot( 2, 2, 2)
    hold on
    plot( data_raw2.x0_arr( :, 1), data_raw2.x0_arr( :, 2),   'linewidth', 2, 'linestyle', '-.', 'color', c.black )
    scatter( 0.5 *( data_raw2.x0i( 1 ) + data_raw2.x0f( 1 ) ), 0.5 *( data_raw2.x0i( 2 ) + data_raw2.x0f( 2 ) ) ...
            , 1000, 'o', 'markerfacecolor', c.grey, 'markeredgecolor', c.black, 'linewidth', 3 )
    set( gca, 'xlim', [-.4, .40001], 'ylim', [ 0.3, 1.8],  'fontsize', 30 )
    % Get the x, y position of the joints 
    q_abs = cumsum( data_raw1.q , 2 );
    x_arr = cumsum( cos( q_abs ), 2 );
    y_arr = cumsum( sin( q_abs ), 2 );

    alpha_arr = [0.2, 0.4, 0.8, 1.0];
    idx_arr   = [100, 1400, 2000, 3000];
    for i = 1 : length( alpha_arr )
        idx = idx_arr( i );
        alpha = alpha_arr( i );
        scatter( [ 0, x_arr( idx, 1:end-1 ) ] , [ 0, y_arr( idx, 1:end-1) ], 200, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
        p2 = plot( [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, :) ], 'color', c.black, 'linewidth', 4 )
        p2.Color( 4 ) = alpha;
        scatter( x_arr( idx, end ), y_arr( idx, end ),  600,  'markerfacecolor', c.blue, 'markeredgecolor', c.blue, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

    end

    plot( data_raw1.p( 1, : ), data_raw1.p( 2, : ), 'linewidth', 4, 'color', c.blue )


    set( gca, 'xlim', [ -1.0, 4.0] , 'ylim', [-1.0, 4.0], 'xticklabel', {}, 'yticklabel', {} )
    axis equal
    title( 'Dynamic Movement Primitives', 'fontsize', 30 )
    set( gca, 'xlim', [-0.2, 0.2] , 'ylim', [0.0, 2.0], 'xticklabel', {}, 'yticklabel', {} )
    axis equal

elseif mode == "MOTOR" || mode == "BOTH" 
    subplot( 2, 2, 4)
    hold on
    plot( data_raw2.x0_arr( :, 1), data_raw2.x0_arr( :, 2),   'linewidth', 2, 'linestyle', '-.', 'color', c.black )
    scatter( 0.5 *( data_raw2.x0i( 1 ) + data_raw2.x0f( 1 ) ), 0.5 *( data_raw2.x0i( 2 ) + data_raw2.x0f( 2 ) ) ...
            , 1000, 'o', 'markerfacecolor', c.grey, 'markeredgecolor', c.black, 'linewidth', 3 )
    set( gca, 'xlim', [-.4, .40001], 'ylim', [ 0.3, 1.8],  'fontsize', 30 )

    % Get the x, y position of the joints 
    q_abs = cumsum( data_raw2.q_arr , 2 );
    x_arr = cumsum( cos( q_abs ), 2 );
    y_arr = cumsum( sin( q_abs ), 2 );

    alpha_arr = [0.2, 0.4, 0.8, 1.0];
    idx_arr   = [100, 1700, 2300, 5000];
    for i = 1 : length( alpha_arr )
        idx = idx_arr( i );
        alpha = alpha_arr( i );
        scatter( [ 0, x_arr( idx, 1:end-1 ) ] , [ 0, y_arr( idx, 1:end-1) ], 200, 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
        p2 = plot( [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, :) ], 'color', c.black, 'linewidth', 4 );
        p2.Color( 4 ) = alpha;
        scatter( x_arr( idx, end ), y_arr( idx, end ),  600,  'markerfacecolor', c.orange, 'markeredgecolor', c.orange, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

    end
    plot( data_raw2.xEE_arr( :, 1 ), data_raw2.xEE_arr( :, 2 ), 'linewidth', 4, 'color', c.orange )


    title( 'Dynamic Motor Primitives', 'fontsize', 30 )
    set( gca, 'xlim', [-0.2, 0.2] , 'ylim', [0.0, 2.0], 'xticklabel', {}, 'yticklabel', {} )
    axis equal
end    

mySaveFig( gcf, 'obstacle' )

