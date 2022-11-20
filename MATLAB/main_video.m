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

% Movement Primitives
file_name2 = '../results/discrete_move_joint_space/movement/dmp1.mat';
file_name3 = '../results/discrete_move_joint_space/movement/dmp2.mat';

data_raw_q1 = load( file_name2 );
data_raw_q2 = load( file_name3 );

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




%% ==================================================================
%% (--) Goal directed Discrete Movement - With Redundancy

% Dynamic Movement Primitives
file_name1 = '../results/discrete_move_task_space_w_redund/movement/dmp.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/discrete_move_task_space_w_redund/motor/ctrl_joint_imp.mat';
file_name3 = '../results/discrete_move_task_space_w_redund/motor/ctrl_task_imp.mat';
data_raw2 = load( file_name2 );
data_raw3 = load( file_name3 );


%% ==================================================================
%% (--) Sequence of Discrete Movements

file_name1 = '../results/sequence/movement/dmp.mat';
file_name2 = '../results/sequence/motor/dmp.mat';

data_raw1 = load( file_name1 );
data_raw2 = load( file_name2 );


%% ==================================================================
%% (--) Task-space Trajectory Tracking --- Rhythmic
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/task_space_traj_track/dynamic_motor/rhythmic.mat';
data_raw1 = load( file_name );



file_name = '../results/task_space_traj_track/dynamic_movement/rhythmic.mat';
data_raw2 = load( file_name );

%% ==================================================================
%% (--) Discrete and Rhythmic Movements 

% Dynamic Movement Primitives
file_name1 = '../results/discrete_and_rhythmic/movement/dmp.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/discrete_and_rhythmic/motor/ctrl_joint_imp.mat';
data_raw2 = load( file_name2 );


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


