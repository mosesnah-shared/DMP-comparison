% [Project]        DMP Comparison
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Oct. 23th, 2022
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION

clear; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );     
myFigureConfig( 'fontsize',  20, ...
               'LineWidth',  10, ...
           'AxesLineWidth', 1.5, ...     For Grid line, axes line width etc.
              'markerSize',  25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor(); 


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
title( 'Dynamic Motor Primitives', 'fontsize', 30 )
axis equal
set( gca, 'xlim', [-0.6, 0.600001], 'ylim', [-0.6, 0.600001] )

mySaveFig( gcf, 'goal_directed_discrete_task_space' )


%% ==================================================================
%% (--) Goal directed Discrete Movement - With Redundancy

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
plot( data_raw1.p( :, 1), data_raw1.p( :, 2  ), 'linewidth', 6, 'color', c.blue )
plot( data_raw3.x0_arr( : , 1 ), data_raw3.x0_arr( : , 2 ), 'linewidth', 6, 'color', c.orange )
set( gca, 'xlim', [-.2, 3.2], 'xtick', [ 0, 1.5, 3.0 ], 'ylim', [ 2.5, 3.5], 'ytick', [ 2.5, 3.0, 3.5 ], 'fontsize', 30 )
legend( '', 'Dynamic Motor Primitives', 'Dynamic Movement Primitives', 'location', 'northwest', 'fontsize', 30  )

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
    p2 = plot( [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, :) ], 'color', c.black, 'linewidth', 4 )
    p2.Color( 4 ) = alpha;
    scatter( x_arr( idx, end ), y_arr( idx, end ),  600,  'markerfacecolor', c.blue, 'markeredgecolor', c.blue, 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    
end

set( gca, 'xlim', [ -1.0, 4.0] , 'ylim', [-1.0, 4.0], 'xticklabel', {}, 'yticklabel', {} )
axis equal
title( 'Dynamic Movement Primitives', 'fontsize', 30 )

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

mySaveFig( gcf, 'redundancy_end_effector' )

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
set( gca, 'fontsize', 30, 'xlim', [1, 5 ], 'ylim', [ -1, 0.5]  )

subplot( 2, 2, 4)
hold on
plot( data_raw2.t_arr, data_raw2.dq_arr )
xlabel( '$t$' )
set( gca, 'fontsize', 30, 'xlim', [1, 5 ], 'ylim', [ -1, 0.5]  )
legend( '$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', 'location', 'southeast')
mySaveFig( gcf, 'redundancy_joint_trajs' )
%% ==================================================================
%% (--) Sequence of Discrete Movements

file_name1 = '../results/sequence/movement/dmp.mat';
file_name2 = '../results/sequence/motor/dmp.mat';

data_raw1 = load( file_name1 );
data_raw2 = load( file_name2 );

figure( )
hold on
plot( data_raw1.t_arr, data_raw1.p( 1, :  ) )
plot( data_raw1.t_arr, data_raw1.p( 2, :  ) )


figure( )
hold on
plot( data_raw2.t_arr, data_raw2.p0_arr( :, 1:2), '--' )
plot( data_raw2.t_arr, data_raw2.dp0_arr( :, 1:2), '--' )

%% ==================================================================
%% (--) Obstacle Avoidance 
%% -- (-A) Dynamic Motor Primitives - Order 

dir_name = '../results/obstacle_avoidance/dynamic_motor/different_order/';

N = 7;
data_raw1 = cell( 1, N );
data_raw2 = cell( 1, N );

for i = 1 : N
    file_name1 = [ dir_name, 'data', num2str( i ), '/ctrl_task_imp.mat' ] ;
    file_name2 = [ dir_name, 'data', num2str( i ), '/ctrl_task_imp2.mat' ] ;
    data_raw1{ i } = load( file_name1 );
    data_raw2{ i } = load( file_name2 );
end
    
f = figure( ); a = axes( 'parent', f );
hold on


idx = find( data_raw1{1}.t_arr - 3 == min( abs( data_raw1{1}.t_arr - 3 ) ));

for i = 1 : N
   plot( data_raw1{ i }.xEE_arr( idx:end ,1 ), data_raw1{ i }.xEE_arr( idx:end ,2 ) , 'linewidth', 7 )
end


tmp = 0.6;
axis equal
set( a, 'xlim', [-0.3, -0.3 + 2*tmp], 'ylim', [0.45, 0.45 + 2*tmp] )

% Draw the obstacle 
obs_pos = data_raw2{1}.obs_pos;
plot( obs_pos( 1 ), obs_pos( 2 ), 'o', 'linewidth', 3, 'color', 'k', 'markeredgecolor', 'k' ,'markerfacecolor', c.grey )
legend( "No $Z_2$", "$n=2$", "$n=3$", "$n=4$", "$n=5$", "$n=6$", "$n=7$", "Obstacle", 'fontsize', 24 )
xlabel( 'x (m)', 'fontsize', 35 )
ylabel( 'y (m)', 'fontsize', 35 )


%% -- (-B) Dynamic Movement Primitives - Order 


