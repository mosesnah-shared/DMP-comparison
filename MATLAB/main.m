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
%% (--) Joint-space Trajectory Tracking - Discrete
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/joint_space_traj_track/dynamic_motor/discrete_movement.mat';

data_raw1 = load( file_name );
data_raw1.q0i = double( data_raw1.q0i );
data_raw1.q0f = double( data_raw1.q0f );
data_raw1.D   = double( data_raw1.D   );
data_raw1.ti  = double( data_raw1.ti  );

f = figure( ); a = axes( 'parent', f );
hold on

[ q0_arr, dq0_arr, ddq0_arr ] = min_jerk_traj( data_raw1.t_arr, data_raw1.q0i, data_raw1.q0f, data_raw1.D, data_raw1.ti );


subplot( 3, 1, 1 )

plot( data_raw1.t_arr, data_raw1.q_arr );
hold on
plot( data_raw1.t_arr, q0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [0, 1.1], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'xticklabel', {},'fontsize', 30 )
ylabel( '$q(t), q_{des}(t)$', 'fontsize', 30 )
legend( '$q(t)$', '$q_{des}(t)$', 'location', 'northwest' )
subplot( 3, 1, 2 )

plot( data_raw1.t_arr, data_raw1.dq_arr );
hold on
plot( data_raw1.t_arr, dq0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [-0.3, 2.3], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'xticklabel', {}, 'fontsize', 30 )
ylabel( '$\dot{q}(t)$, $\dot{q}_{des}(t)$', 'fontsize', 30 )
legend( '$\dot{q}(t)$', '$\dot{q}_{des}(t)$', 'location', 'northwest' )
subplot( 3, 1, 3 )
plot( data_raw1.t_arr, data_raw1.ddq_arr );
hold on
plot( data_raw1.t_arr, ddq0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [-7, 7], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'fontsize', 30 )

ylabel( '$\ddot{q}(t)$, $\ddot{q}_{des}(t)$', 'fontsize', 30 )
xlabel( 'Time (s)', 'fontsize', 30 )
legend( '$\ddot{q}(t)$', '$\ddot{q}_{des}(t)$', 'location', 'southwest' )
currentFigure = gcf;
title( currentFigure.Children(end), 'Dynamic Motor Primitives');
mySaveFig( gcf, 'discrete_motor_primitives' )

%% -- (-B) Dynamic Movement Primitives 

file_name = '../results/joint_space_traj_track/dynamic_movement/discrete_movement.mat';
data_raw2 = load( file_name );

subplot( 3, 1, 1 )
hold on
plot( data_raw1.t_arr, data_raw1.q_arr, 'linewidth', 8 );
plot( data_raw2.t_arr, data_raw2.y_arr, 'linewidth', 8 );
plot( data_raw2.t_des, data_raw2.y_des, 'linewidth', 8, 'linestyle', '--', 'color', c.black );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [0, 1.1], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'xticklabel', {},'fontsize', 30 )
ylabel( '$q(t), q_{des}(t)$', 'fontsize', 30 )
% legend( 'Dynamic Motor Primitives', 'Dynamic Movement Primitives', '$q_{des}(t)$', 'location', 'northwestoutside' )

subplot( 3, 1, 2 )
hold on
plot( data_raw1.t_arr, data_raw1.dq_arr, 'linewidth', 8 );
plot( data_raw2.t_arr, data_raw2.z_arr / data_raw2.tau, 'linewidth', 8 );
plot( data_raw2.t_des, data_raw2.dy_des, 'linewidth', 8, 'linestyle', '--', 'color', c.black );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [-0.3, 2.3], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'xticklabel', {}, 'fontsize', 30 )
ylabel( '$\dot{q}(t)$, $\dot{q}_{des}(t)$', 'fontsize', 30 )
% legend( '$\dot{q}(t)$', '$\dot{q}_{des}(t)$', 'location', 'northwest' )

subplot( 3, 1, 3 )
hold on
plot( data_raw1.t_arr, data_raw1.ddq_arr , 'linewidth', 8 );
plot( data_raw2.t_arr, data_raw2.dz_arr , 'linewidth', 8 );
plot( data_raw2.t_des, data_raw2.ddy_des, 'linewidth', 8, 'linestyle', '--', 'color', c.black );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [-7, 7], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'fontsize', 30 )

ylabel( '$\ddot{q}(t)$, $\ddot{q}_{des}(t)$', 'fontsize', 30 )
xlabel( 'Time (s)', 'fontsize', 30 )
% legend( '$\ddot{q}(t)$', '$\ddot{q}_{des}(t)$', 'location', 'southwest' )

mySaveFig( gcf, 'joint_space_discrete_trajectory' )

%% -- (-C) Comparison of the Three approaches

subplot( 3, 1, 1 )

plot( data_raw2.t_arr, data_raw2.y_arr );
hold on
plot( data_raw2.t_des, data_raw2.y_des, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [0, 1.1], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'xticklabel', {},'fontsize', 30 )
ylabel( '$q(t), q_{des}(t)$', 'fontsize', 30 )
legend( '$q(t)$', '$q_{des}(t)$', 'location', 'northwest' )
subplot( 3, 1, 2 )

plot( data_raw2.t_arr, data_raw2.z_arr / data_raw2.tau );
hold on
plot( data_raw2.t_des, data_raw2.dy_des, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [-0.3, 2.3], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'xticklabel', {}, 'fontsize', 30 )
ylabel( '$\dot{q}(t)$, $\dot{q}_{des}(t)$', 'fontsize', 30 )
legend( '$\dot{q}(t)$', '$\dot{q}_{des}(t)$', 'location', 'northwest' )

subplot( 3, 1, 3 )
plot( data_raw2.t_arr, data_raw2.dz_arr );
hold on
plot( data_raw2.t_des, data_raw2.ddy_des, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 1.3 ], 'ylim', [-7, 7], 'xtick', [ 0, 0.5, 1.0, 1.3 ], 'fontsize', 30 )

ylabel( '$\ddot{q}(t)$, $\ddot{q}_{des}(t)$', 'fontsize', 30 )
xlabel( 'Time (s)', 'fontsize', 30 )
legend( '$\ddot{q}(t)$', '$\ddot{q}_{des}(t)$', 'location', 'southwest' )
currentFigure = gcf;
title( currentFigure.Children(end), 'Dynamic Movement Primitives');
mySaveFig( gcf, 'discrete_movement_primitives' )

%% ==================================================================
%% (--) Joint-space Trajectory Tracking - Rhythmic
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/joint_space_traj_track/dynamic_motor/rhythmic_movement.mat';

data_raw1 = load( file_name );

f = figure( ); a = axes( 'parent', f );
hold on


subplot( 3, 1, 1 )

plot( data_raw1.t_arr, data_raw1.q_arr );
hold on
plot( data_raw1.t_arr, data_raw1.q0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$q(t), q_{des}(t)$', 'fontsize', 30 )
legend( '$q(t)$', '$q_{des}(t)$', 'location', 'southwest' )
subplot( 3, 1, 2 )

plot( data_raw1.t_arr, data_raw1.dq_arr );
hold on
plot( data_raw1.t_arr, data_raw1.dq0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$\dot{q}(t)$, $\dot{q}_{des}(t)$', 'fontsize', 30 )
legend( '$\dot{q}(t)$', '$\dot{q}_{des}(t)$', 'location', 'southwest' )
subplot( 3, 1, 3 )
plot( data_raw1.t_arr, data_raw1.ddq_arr );
hold on
plot( data_raw1.t_arr, data_raw1.ddq0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )

ylabel( '$\ddot{q}(t)$, $\ddot{q}_{des}(t)$', 'fontsize', 30 )
xlabel( 'Time (s)', 'fontsize', 30 )
legend( '$\ddot{q}(t)$', '$\ddot{q}_{des}(t)$', 'location', 'northwest' )
currentFigure = gcf;
title( currentFigure.Children(end), 'Dynamic Motor Primitives');
mySaveFig( gcf, 'discrete_motor_primitives' )

%% -- (-B) Dynamic Movement Primitives 

file_name = '../results/joint_space_traj_track/dynamic_movement/rhythmic_movement.mat';
data_raw2 = load( file_name );
subplot( 3, 1, 1 )

plot( data_raw2.t_arr, data_raw2.y_arr );
hold on
plot( data_raw1.t_arr, data_raw1.q0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$q(t), q_{des}(t)$', 'fontsize', 30 )
legend( '$q(t)$', '$q_{des}(t)$', 'location', 'northwest' )
subplot( 3, 1, 2 )

plot( data_raw2.t_arr, data_raw2.z_arr / data_raw2.tau );
hold on
plot( data_raw1.t_arr, data_raw1.dq0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$\dot{q}(t)$, $\dot{q}_{des}(t)$', 'fontsize', 30 )
legend( '$\dot{q}(t)$', '$\dot{q}_{des}(t)$', 'location', 'northwest' )

subplot( 3, 1, 3 )
plot( data_raw2.t_arr, data_raw2.dz_arr );
hold on
plot( data_raw1.t_arr, data_raw1.ddq0_arr, 'linewidth', 4, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )

ylabel( '$\ddot{q}(t)$, $\ddot{q}_{des}(t)$', 'fontsize', 30 )
xlabel( 'Time (s)', 'fontsize', 30 )
legend( '$\ddot{q}(t)$', '$\ddot{q}_{des}(t)$', 'location', 'southwest' )
currentFigure = gcf;
title( currentFigure.Children(end), 'Dynamic Movement Primitives');
mySaveFig( gcf, 'discrete_movement_primitives' )

%% -- (-C) Comparison of the Three approaches

subplot( 3, 1, 1 )
hold on
plot( data_raw1.t_arr, data_raw1.q_arr, 'linewidth', 8 );
plot( data_raw2.t_arr, data_raw2.y_arr, 'linewidth', 8 );
plot( data_raw1.t_arr, data_raw1.q0_arr, 'linewidth', 8, 'linestyle', '--', 'color', c.black );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$q(t), q_{des}(t)$', 'fontsize', 30 )
% legend( 'Dynamic Motor Primitives', 'Dynamic Movement Primitives', '$q_{des}(t)$', 'location', 'northwestoutside' )

subplot( 3, 1, 2 )
hold on
plot( data_raw1.t_arr, data_raw1.dq_arr, 'linewidth', 8 );
plot( data_raw2.t_arr, data_raw2.z_arr / data_raw2.tau, 'linewidth', 8 );
plot( data_raw1.t_arr, data_raw1.dq0_arr, 'linewidth', 8, 'linestyle', '--', 'color', c.black );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$\dot{q}(t)$, $\dot{q}_{des}(t)$', 'fontsize', 30 )
% legend( '$\dot{q}(t)$', '$\dot{q}_{des}(t)$', 'location', 'northwest' )

subplot( 3, 1, 3 )
hold on
plot( data_raw1.t_arr, data_raw1.ddq_arr , 'linewidth', 8 );
plot( data_raw2.t_arr, data_raw2.dz_arr , 'linewidth', 8 );
plot( data_raw1.t_arr, data_raw1.ddq0_arr, 'linewidth', 8, 'linestyle', '--', 'color', c.black );
set( gca, 'xlim', [ 0, 4.0 ], 'fontsize', 30 )
ylabel( '$\ddot{q}(t)$, $\ddot{q}_{des}(t)$', 'fontsize', 30 )
xlabel( 'Time (s)', 'fontsize', 30 )
% legend( '$\ddot{q}(t)$', '$\ddot{q}_{des}(t)$', 'location', 'southwest' )

mySaveFig( gcf, 'joint_space_rhythmic_trajectory' )



%% ==================================================================
%% (--) Task-space Trajectory Tracking --- Discrete
%% -- (-A) Dynamic Motor Primitives 

% The number of targets
N = 8;

data_raw1 = cell( 8 ); 

c_arr = [ c.blue; c.orange; c.green; c.purple; c.roseRed; c.peach; c.pink; c.blue_sky ];

for i = 1 : 8
    file_name = ['../results/task_space_traj_track/dynamic_motor/target', num2str( i ), '.mat' ];
    data_raw1{ i } = load( file_name );
end

f = figure( ); a = axes( 'parent', f );
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
set( a, 'xlim', [-0.6, 0.600001], 'ylim', [-0.6, 0.600001] )

mySaveFig( gcf,  'task_space_discrete_motor' )

%% -- (-B) Dynamic Movement Primitives 

% The number of targets
N = 8;

data_raw2 = cell( 8 ); 

c_arr = [ c.blue; c.orange; c.green; c.purple; c.roseRed; c.peach; c.pink; c.blue_sky ];

for i = 1 : 8
    file_name = ['../results/task_space_traj_track/dynamic_movement/target', num2str( i ), '.mat' ];
    data_raw2{ i } = load( file_name );
end


f = figure( ); a = axes( 'parent', f );
hold on

xEEi = [ data_raw2{ i }.y_des1( 1 ), data_raw2{ i }.y_des2( 1 )];
r = 0.5;
for i = 1 : 8
    plot( r * cos( ( i - 1 ) * pi/4 ) ,  r * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 30, 'markerfacecolor', c_arr( i, : ), 'markeredgecolor', c_arr( i, : ), 'linewidth', 3 )
    plot( data_raw2{ i }.y_arr1 - xEEi( 1 ), data_raw2{ i }.y_arr2 - xEEi( 2 ), 'linewidth', 8, 'color', c_arr( i, : ) )
    plot( data_raw2{ i }.y_des1 - xEEi( 1 ), data_raw2{ i }.y_des2 - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
end

xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )

axis equal
set( a, 'xlim', [-0.6, 0.600001], 'ylim', [-0.6, 0.600001] )

mySaveFig( gcf,  'task_space_discrete_movement' )

%% ==================================================================
%% (--) Task-space Trajectory Tracking --- Rhythmic
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/task_space_traj_track/dynamic_motor/rhythmic.mat';
data_raw1 = load( file_name );

f = figure( ); a = axes( 'parent', f );
hold on

centers = data_raw1.centers;

plot( data_raw1.xEE_arr( :, 1 )-centers(1), data_raw1.xEE_arr( :, 2 )- centers(2), 'linewidth', 6, 'color', c.orange )
plot( data_raw1.x0_arr( :, 1 )-centers(1), data_raw1.x0_arr( :, 2 )- centers(2), 'linewidth', 3, 'linestyle', '--', 'color', 'k')
 
xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
axis equal
set( a, 'xlim', [-0.6, 0.60001], 'ylim', [-0.6, 0.60001] )
title( 'Dynamic Motor Primitives', 'fontsize', 30 )
mySaveFig( gcf,  'task_space_rhythmic_motor' )


%% -- (-B) Dynamic Movement Primitives 

file_name = '../results/task_space_traj_track/dynamic_movement/rhythmic.mat';
data_raw2 = load( file_name );

f = figure( ); a = axes( 'parent', f );
hold on


plot( data_raw2.y_arr1-centers(1), data_raw2.y_arr2-centers(2), 'linewidth', 6, 'color', c.orange )
plot( data_raw2.y_des1-centers(1), data_raw2.y_des2-centers(2), 'linewidth', 3, 'linestyle', '--', 'color', 'k')
 
xlabel( '$X$ (m)', 'fontsize', 30 )
ylabel( '$Y$ (m)', 'fontsize', 30 )
axis equal
set( a, 'xlim', [-0.6, 0.60001], 'ylim', [-0.6, 0.60001] )
title( 'Dynamic Movement Primitives', 'fontsize', 30 )
mySaveFig( gcf,  'task_space_rhythmic_movement' )

%% -- (-C) Comparison of the Three approaches -- Task-space


%% -- (-D) Comparison of the Three approaches -- Joint-space


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
