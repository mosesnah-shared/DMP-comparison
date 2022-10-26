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
%% (--) Joint-space Trajectory Tracking
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/joint_space_traj_track/dynamic_motor/ctrl_joint_imp.mat';

data_raw = load( file_name );
data_raw.q0i = double( data_raw.q0i );
data_raw.q0f = double( data_raw.q0f );
data_raw.D   = double( data_raw.D   );
data_raw.ti  = double( data_raw.ti  );

%% -- (-B) Dynamic Movement Primitives 

f = figure( ); a = axes( 'parent', f );
hold on

[ q0_arr, dq0_arr, ddq0_arr ] = min_jerk_traj( data_raw.t_arr, data_raw.q0i, data_raw.q0f, data_raw.D, data_raw.ti );

subplot( 3, 1, 1 )
plot( data_raw.t_arr, data_raw.q_arr );
hold on
plot( data_raw.t_arr, q0_arr, 'linewidth', 4, 'linestyle', '--' );

subplot( 3, 1, 2 )
plot( data_raw.t_arr, data_raw.dq_arr );
hold on
plot( data_raw.t_arr, dq0_arr, 'linewidth', 4, 'linestyle', '--' );

subplot( 3, 1, 3 )
plot( data_raw.t_arr, data_raw.ddq_arr );
hold on
plot( data_raw.t_arr, ddq0_arr, 'linewidth', 4, 'linestyle', '--' );


%% ==================================================================
%% (--) Task-space Trajectory Tracking
%% -- (-A) Dynamic Motor Primitives 





%% -- (-B) Dynamic Movement Primitives 







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
%% -- (1A) Dynamic Motor Primitives - k values