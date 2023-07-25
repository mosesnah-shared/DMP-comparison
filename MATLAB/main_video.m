% [Project]        DMP Comparison
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
               'LineWidth',  2, ...
           'AxesLineWidth', 1.5, ...     For Grid line, axes line width etc.
              'markerSize',  5    )  
             
global c        
c  = myColor(); 

global mode 
mode = "MOTOR"; % Either movement or motor or both to generate the images 

%% ==================================================================
%% (--) Goal directed Discrete Movement - Joint Space

clear gObjs
TASK = "_DISCRETE_JOINT_SPACE";

% Motor Primitives
file_name1 = '../results/discrete_move_joint_space/motor/ctrl_joint_imp.mat';
data_raw_q1 = load( file_name1 );

% Movement Primitives
file_name2 = '../results/discrete_move_joint_space/movement/ctrl_joint_dmp.mat';
data_raw_q2 = load( file_name2 );


if mode == "MOVEMENT"
   
    dt = data_raw_q2.t_arr( 2 ) - data_raw_q2.t_arr( 1 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw_q2.q_arr, 2 );
    q0_abs = cumsum( data_raw_q2.q_command', 2 );
    
    
    xSH = zeros( 1, length( data_raw_q2.t_arr ) );
    ySH = zeros( 1, length( data_raw_q2.t_arr ) );
    zSH = zeros( 1, length( data_raw_q2.t_arr ) );
    
    jSize = 1400;
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 1.0       );          
    
    % Elbow
    xEL = cos( q_abs( 1, : )' );
    yEL = sin( q_abs( 1, : )' );
    zEL = zSH;
    

    % Elbow
    x0EL = cos( q0_abs( 1, : )' );
    y0EL = sin( q0_abs( 1, : )' );
    z0EL = zSH;
        
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 1.0       );      
              
    gObjs( 3 ) = myMarker( 'XData', x0EL, 'YData', y0EL, 'ZData', z0EL, ... 
                             'name', "EL0"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3       );                    


    % Shoulder
    xEE = xEL + cos( q_abs( 2, : )' );
    yEE = yEL + sin( q_abs( 2, : )' );
    zEE = zSH;
    
    x0EE = x0EL + cos( q0_abs( 2, : )' );
    y0EE = y0EL + sin( q0_abs( 2, : )' );
    z0EE = zSH;    
              
              
    % End Effector
    gObjs( 4 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 1.0       );     

    gObjs( 5 ) = myMarker( 'XData', x0EE, 'YData', y0EE, 'ZData', z0EE, ... 
                             'name', "EE0"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3       );                    

              
              
    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL0",     "EE0" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '--', 'LineWidth', 2 ); 

    ani.adjustFigures( 2 );                     
    a2 = ani.hAxes{ 2 };
    plot( data_raw_q2.t_arr, data_raw_q2.q_command( 1, : ),'linestyle', '--', 'color', c.black, 'linewidth', 2)
    plot( data_raw_q2.t_arr, data_raw_q2.q_arr( 1, : ), 'color', c.purple, 'linewidth', 5 )
    set( a2, 'fontsize', 30, 'xlim' ,[0, 3], 'ylim', [-0.5, 1.5] )
    tmp = myMarker( 'XData', data_raw_q2.t_arr, 'YData', data_raw_q2.q_arr( :, 1 )' , 'ZData', zeros( 1, length( data_raw_q2.t_arr ) ), ...
                    'SizeData',  700, 'LineWidth', 5 , 'MarkerEdgeColor',  c.purple ); 
    ani.addTrackingPlots( 2, tmp );
    ylabel( ani.hAxes{ 2 }, '$q_{shoulder} (t)$', 'fontsize', 30 )
    % xlabel( ani.hAxes{ 2 }, '$t$', 'fontsize', 30 )

    ani.adjustFigures( 3 );                     
    a3 = ani.hAxes{ 3 };
    plot( data_raw_q2.t_arr, data_raw_q2.q_command( 2, : ),'linestyle', '--', 'color', c.black, 'linewidth', 2)
    plot( data_raw_q2.t_arr, data_raw_q2.q_arr( 2, : ), 'color', c.purple, 'linewidth', 5 )
    set( a3, 'fontsize', 30, 'xlim' ,[0, 3], 'ylim', [-0.5, 1.5] )
    tmp = myMarker( 'XData', data_raw_q2.t_arr, 'YData', data_raw_q2.q_arr( :, 2 )' , 'ZData', zeros( 1, length( data_raw_q2.t_arr ) ), ...
                    'SizeData',  700, 'LineWidth', 5 , 'MarkerEdgeColor',  c.purple ); 
    ani.addTrackingPlots( 3, tmp );
    ylabel( ani.hAxes{ 3 }, '$q_{elbow} (t)$', 'fontsize', 30 )

    tmpLim = 1.2;
    cen = [ 0.0, 2.0 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 2 )] , ...                  
                         'YLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 2 ) ] , ...    
                         'ZLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 2 ) ] , ...
                         'view',   [0, 90]    , 'fontsize', 30 )      
    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )

    ani.run( 0.5, 0.0, 3.0, true, strcat( mode, TASK )     )

              
elseif mode == "MOTOR"

    dt = data_raw_q1.t_arr( 2 ) - data_raw_q1.t_arr( 1 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw_q1.q_arr, 1 );
    q0_abs = cumsum( data_raw_q1.q0_arr, 1 );
    
    
    xSH = zeros( 1, length( data_raw_q1.t_arr ) );
    ySH = zeros( 1, length( data_raw_q1.t_arr ) );
    zSH = zeros( 1, length( data_raw_q1.t_arr ) );
    
    jSize = 1400;
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                     
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    % Elbow
    x0EL = cos( q0_abs( 1,: ) );
    y0EL = sin( q0_abs( 1,: ) );
    z0EL = zSH;
        
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );      
              
    gObjs( 3 ) = myMarker( 'XData', x0EL, 'YData', y0EL, 'ZData', z0EL, ... 
                             'name', "EL0"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3       );                    


    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
    
    x0EE = x0EL + cos( q0_abs( 2, : ) );
    y0EE = y0EL + sin( q0_abs( 2, : ) );
    z0EE = zSH;    
              
              
    % End Effector
    gObjs( 4 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );     

    gObjs( 5 ) = myMarker( 'XData', x0EE, 'YData', y0EE, 'ZData', z0EE, ... 
                             'name', "EE0"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3       );                    

              
              
    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL0",   "EE0" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '--', 'LineWidth', 2 ); 


    ani.adjustFigures( 2 );                     
    a2 = ani.hAxes{ 2 };
    plot( data_raw_q1.t_arr, data_raw_q1.q0_arr( 1, : ),'linestyle', '--', 'color', c.black , 'linewidth', 2)
    plot( data_raw_q1.t_arr, data_raw_q1.q_arr( 1, : ), 'color', c.orange , 'linewidth', 5)
    set( a2, 'fontsize', 30, 'xlim' ,[0, 3], 'ylim', [-0.5, 1.5], 'xticklabel', [ "0", "1", "2", "3" ] )
    tmp = myMarker( 'XData', data_raw_q1.t_arr, 'YData', data_raw_q1.q_arr( 1, : ) , 'ZData', zeros( 1, length( data_raw_q1.t_arr ) ), ...
                    'SizeData',  700, 'LineWidth', 5 , 'MarkerEdgeColor',  c.orange ); 
    ani.addTrackingPlots( 2, tmp );
    ylabel( ani.hAxes{ 2 }, '$q_{shoulder} (t)$', 'fontsize', 30 )
    % xlabel( ani.hAxes{ 2 }, '$t$', 'fontsize', 30 )

    ani.adjustFigures( 3 );                     
    a3 = ani.hAxes{ 3 };
    plot( data_raw_q1.t_arr, data_raw_q1.q0_arr( 2, : ),'linestyle', '--', 'color', c.black, 'linewidth', 2)
    plot( data_raw_q1.t_arr, data_raw_q1.q_arr( 2, : ), 'color', c.orange, 'linewidth', 5 )
    set( a3, 'fontsize', 30, 'xlim' ,[0, 3], 'ylim', [-0.5, 1.5], 'xticklabel', [ "0", "1", "2", "3" ] )
    tmp = myMarker( 'XData', data_raw_q1.t_arr, 'YData', data_raw_q1.q_arr( 2, : ), 'ZData', zeros( 1, length( data_raw_q1.t_arr ) ), ...
                    'SizeData',  700, 'LineWidth', 5 , 'MarkerEdgeColor',  c.orange ); 
    ani.addTrackingPlots( 3, tmp );
    ylabel( ani.hAxes{ 3 }, '$q_{elbow} (t)$', 'fontsize', 30 )

    tmpLim = 1.2;
    cen = [ 0.0, 2.0 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 2 )] , ...                  
                         'YLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 2 ) ] , ...    
                         'ZLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 2 ) ] , ...
                         'view',   [0, 90]    , 'fontsize', 30 )      
    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )

    ani.run( 0.7  , 0.0, 3.0, true, strcat( mode, TASK )     )
              
              
end


%% ==================================================================
%% (--) Goal directed Discrete Movement - Task-Space


clear gObjs data*

TASK = "_DISCRETE_TASK_SPACE";


% The number of targets
N = 8;
data_raw1 = cell( 1,8 ); 

idx = 7; 

c_arr = [ c.blue; c.orange; c.green; c.orange; c.roseRed; c.peach; c.pink; c.blue_sky ];

if ( mode == "MOTOR" || mode == "BOTH")
    for i = 1 : 8
        file_name = ['../results/discrete_move_task_space_wo_redund/motor/target', num2str( i ), '/ctrl_task_imp.mat' ];
        data_raw1{ i  } = load( file_name );
    end
end

if ( mode == "MOVEMENT" || mode == "BOTH")
    for i = 1 : 8
        file_name = ['../results/discrete_move_task_space_wo_redund/movement/target', num2str( i ), '.mat' ];
        data_raw2{ i } = load( file_name );
    end
end 

xEEi = data_raw1{ idx }.p_arr( :, 1 );


if mode == "MOVEMENT"
    data_raw = data_raw2{ idx };
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    % Shoulder
    jSize = 1400;
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                     
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 

    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              

    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 1.8;
    cen = [ -0.0, 1.2 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim+cen( 1 ), tmpLim+cen( 1 )] , ...                  
                         'YLim',   [ -tmpLim+cen( 2 ), tmpLim+cen( 2 )] , ...    
                         'ZLim',   [ -tmpLim, tmpLim] , ...
                         'view',   [0, 90], 'fontsize', 30  )      
    for i = 1 : 8
        plot( ani.hAxes{1}, xEEi( 1 ) + 0.5 * cos( ( i - 1 ) * pi/4 ) , xEEi( 2 ) + 0.5 * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 8, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 3 )
    %     plot( ani.hAxes{1}, data_raw2{ i }.x , data_raw2{ i }.y , 'linewidth', 3, 'color', c_arr( i, : ) )
    %     plot( ani.hAxes{1}, data_raw1{ i }.x0_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.x0_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
    end                 

    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )


    ani.run( 0.50, 0.0, 1.5, true, strcat( mode, TASK, num2str( idx ) )     )    

              
elseif mode == "MOTOR"

    data_raw = data_raw1{ idx };
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    % Shoulder
    jSize = 1400;
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                     
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 

    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              

    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 1.8;
    cen = [ -0.0, 1.2 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim+cen( 1 ), tmpLim+cen( 1 )] , ...                  
                         'YLim',   [ -tmpLim+cen( 2 ), tmpLim+cen( 2 )] , ...    
                         'ZLim',   [ -tmpLim, tmpLim] , ...
                         'view',   [0, 90], 'fontsize', 30  )      
    for i = 1 : 8
        plot( ani.hAxes{1}, xEEi( 1 ) + 0.5 * cos( ( i - 1 ) * pi/4 ) , xEEi( 2 ) + 0.5 * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 8, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 3 )
    %     plot( ani.hAxes{1}, data_raw2{ i }.x , data_raw2{ i }.y , 'linewidth', 3, 'color', c_arr( i, : ) )
    %     plot( ani.hAxes{1}, data_raw1{ i }.x0_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.x0_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
    end                 

    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )


    ani.run( 1.0, 0.0, 1.5, true, strcat( mode, TASK, num2str( idx ) )     )
              
              
end



%% ==================================================================
%% (--) Goal directed Discrete Movement - Task-Space, ALL MOVEMENTS


clear gObjs data*

TASK = "_DISCRETE_TASK_SPACE";

% The number of targets
N = 8;
data_raw1 = cell( 1,8 ); 

idx = 7; 

c_arr = [ c.blue; c.orange; c.green; c.orange; c.roseRed; c.peach; c.pink; c.blue_sky ];

if ( mode == "MOTOR" || mode == "BOTH")
    for i = 1 : 8
        file_name = ['../results/discrete_move_task_space_wo_redund/motor/target', num2str( i ), '/ctrl_task_imp.mat' ];
        data_raw1{ i  } = load( file_name );
    end
end

if ( mode == "MOVEMENT" || mode == "BOTH")
    for i = 1 : 8
        file_name = ['../results/discrete_move_task_space_wo_redund/movement/target', num2str( i ), '.mat' ];
        data_raw2{ i } = load( file_name );
    end
end 

xEEi = data_raw1{ idx }.p_arr( :, 1 );


if mode == "MOVEMENT"
    data_raw = data_raw2{ idx };
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    % Shoulder
    jSize = 1400;
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                     
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 

    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              

    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 1.8;
    cen = [ -0.0, 1.2 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim+cen( 1 ), tmpLim+cen( 1 )] , ...                  
                         'YLim',   [ -tmpLim+cen( 2 ), tmpLim+cen( 2 )] , ...    
                         'ZLim',   [ -tmpLim, tmpLim] , ...
                         'view',   [0, 90], 'fontsize', 30  )      
    for i = 1 : 8
        plot( ani.hAxes{1}, xEEi( 1 ) + 0.5 * cos( ( i - 1 ) * pi/4 ) , xEEi( 2 ) + 0.5 * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 8, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 3 )
    %     plot( ani.hAxes{1}, data_raw2{ i }.x , data_raw2{ i }.y , 'linewidth', 3, 'color', c_arr( i, : ) )
    %     plot( ani.hAxes{1}, data_raw1{ i }.x0_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.x0_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
    end                 

    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )


    ani.run( 0.50, 0.0, 1.5, true, strcat( mode, TASK, num2str( idx ) )     )    

              
elseif mode == "MOTOR"

    data_raw = data_raw1{ 1 };
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );

    % Shoulder
    jSize = 1400;

    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
        
    
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                        'LineWidth',   8       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0       );          
              
    for i = 1 : 8
        
        % Change to absolute angle
        q_abs = cumsum( data_raw1{ i }.q_arr, 1 );

        % Elbow
        xEL = cos( q_abs( 1, : ) );
        yEL = sin( q_abs( 1, : ) );
        zEL = zSH;

        gObjs( 2*i ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                                 'name', ['EL', num2str( i ) ]  , 'SizeData',  jSize*0.5, ...
                            'LineWidth',   5       , ...
                      'MarkerEdgeColor', c.orange, ...
                      'MarkerFaceColor', c.white, ...
                      'MarkerFaceAlpha', 1.0       ); 

        xEE = xEL + cos( q_abs( 2, : ) );
        yEE = yEL + sin( q_abs( 2, : ) );
        zEE = zSH;


        % End Effector
        gObjs( 2*i+1 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                                 'name', ['EE', num2str( i ) ]  , 'SizeData',  jSize      , ...
                            'LineWidth',   5       , ...
                      'MarkerEdgeColor', c.orange, ...
                      'MarkerFaceColor', c.white, ...
                      'MarkerFaceAlpha', 1.0       ); 
                 
        
    end
             

    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL1",     "EE1" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL2",     "EE2" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL3",     "EE3" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL4",     "EE4" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL5",     "EE5" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL6",     "EE6" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL7",     "EE7" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL8",     "EE8" ], 'Color', [ 0.5, 0.5, 0.5 ], 'LineStyle',  '-' ); 

    tmpLim = 1.8;
    cen = [ -0.0, 1.2 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim+cen( 1 ), tmpLim+cen( 1 )] , ...                  
                         'YLim',   [ -tmpLim+cen( 2 ), tmpLim+cen( 2 )] , ...    
                         'ZLim',   [ -tmpLim, tmpLim] , ...
                         'view',   [0, 90], 'fontsize', 30  )      
    for i = 1 : 8
        plot( ani.hAxes{1}, xEEi( 1 ) + 0.5 * cos( ( i - 1 ) * pi/4 ) , xEEi( 2 ) + 0.5 * sin( ( i - 1 ) * pi/4 ), 'o', 'markersize', 8, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 3 )
    %     plot( ani.hAxes{1}, data_raw2{ i }.x , data_raw2{ i }.y , 'linewidth', 3, 'color', c_arr( i, : ) )
    %     plot( ani.hAxes{1}, data_raw1{ i }.x0_arr( :, 1 ) - xEEi( 1 ), data_raw1{ i }.x0_arr( :, 2 ) - xEEi( 2 ), 'linewidth', 3, 'color', c.black, 'linestyle',  '--' )
    end                 

    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )


    ani.run( .5, 0.0, 1.5, true, strcat( mode, TASK, num2str( idx ) )     )
              
              
end




%% ==================================================================
%% (--) Goal directed Discrete Movement - With Redundancy

% Dynamic Movement Primitives
file_name1 = '../results/discrete_move_task_space_w_redund/movement/ctrl_task_dmp.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/discrete_move_task_space_w_redund/motor/ctrl_joint_imp.mat';
file_name3 = '../results/discrete_move_task_space_w_redund/motor/ctrl_task_imp.mat';
data_raw2 = load( file_name2 );
data_raw3 = load( file_name3 );


clear gObjs 

TASK = "_REDUNDANCY";


if mode == "MOVEMENT"
    
    data_raw = data_raw1;
              
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    % Get the number of joints
    [ ~, nJ ] = size( q_abs );

    x = zeros( 1, length( data_raw.t_arr ) );
    y = zeros( 1, length( data_raw.t_arr ) );
    z = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;
            
    gObjs(  1 ) = myMarker( 'XData', x, 'YData', y, 'ZData', z, ... 
                             'name', strcat( "joint1" ), 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              
    
    for i = 2:6

        x = x + cos( q_abs( i - 1, : ) );
        y = y + sin( q_abs( i - 1, : ) );
        z = z;
    
        
        gObjs(  i) = myMarker( 'XData', x, 'YData', y, 'ZData', z, ... 
                                 'name', strcat( "joint", num2str( i ) )  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
                       
    
    end

    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 3.5;
    cen = [ 1.0, 2.0 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                         'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ...    
                         'ZLim',   [ -tmpLim , tmpLim ] , ...
                         'view',   [0, 90], 'fontsize', 30  )      

    scatter( ani.hAxes{ 1 }, data_raw3.x0_arr( 1, 1 ), data_raw3.x0_arr( 2, 1 ), 300, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
    scatter( ani.hAxes{ 1 }, data_raw3.x0_arr( 1, end ), data_raw3.x0_arr( 2, end ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )

    plot( [ data_raw3.x0_arr( 1 , 1 ), data_raw3.x0_arr( 1, end ) ], [ data_raw3.x0_arr( 2,1  ),  data_raw3.x0_arr( 2, end )] , 'linewidth', 3, 'linestyle', '--', 'color', c.black )

    ani.adjustFigures( 3 );      
    
    a3 = ani.hAxes{ 3 };
    
    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )
        
    plot( a3, data_raw1.t_arr, data_raw1.dq_arr, 'linewidth', 5 )
    set( a3, 'fontsize', 30, 'xlim', [0, 3 ], 'ylim', [ -1, 1 ]  )
    title( 'Dynamic Movement Primitives', 'fontsize', 30 )

    tmp = myMarker( 'XData', data_raw1.t_arr, 'YData', data_raw1.dq_arr( :, 1 ), 'ZData', zeros( 1, length( data_raw1.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  c.blue ); 
    ani.addTrackingPlots( 3, tmp );
    
    tmp = myMarker( 'XData', data_raw1.t_arr, 'YData', data_raw1.dq_arr( :, 2 ), 'ZData', zeros( 1, length( data_raw1.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  c.orange ); 
    ani.addTrackingPlots( 3, tmp );    
    
    tmp = myMarker( 'XData', data_raw1.t_arr, 'YData', data_raw1.dq_arr( :, 3 ), 'ZData', zeros( 1, length( data_raw1.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  c.yellow ); 
    ani.addTrackingPlots( 3, tmp );        
    
    tmp = myMarker( 'XData', data_raw1.t_arr, 'YData', data_raw1.dq_arr( :, 4 ), 'ZData', zeros( 1, length( data_raw1.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  c.orange ); 
    ani.addTrackingPlots( 3, tmp );            
    
    tmp = myMarker( 'XData', data_raw1.t_arr, 'YData', data_raw1.dq_arr( :, 5 ), 'ZData', zeros( 1, length( data_raw1.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  c.green ); 
    ani.addTrackingPlots( 3, tmp );                
    
    xlabel( ani.hAxes{ 3 }, 'Time (s)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 3 }, 'Joint Velicity (rad/s)', 'fontsize', 30 )
    

    ani.run( 0.50, 0.0, 3.0, true, strcat( mode, TASK )     )

   
    
elseif mode == "MOTOR"

    data_raw = data_raw2;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    % Get the number of joints
    [ ~, nJ ] = size( q_abs );

    x = zeros( 1, length( data_raw.t_arr ) );
    y = zeros( 1, length( data_raw.t_arr ) );
    z = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;
            
    gObjs(  1 ) = myMarker( 'XData', x, 'YData', y, 'ZData', z, ... 
                             'name', strcat( "joint1" ), 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              
    
    for i = 2:6

        x = x + cos( q_abs( i - 1, : ) );
        y = y + sin( q_abs( i - 1, : ) );
        z = z;
        
        gObjs(  i) = myMarker( 'XData', x, 'YData', y, 'ZData', z, ... 
                                 'name', strcat( "joint", num2str( i ) )  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );          
    
    end

    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 3.5;
    cen = [ 1.0, 2.0 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                         'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ...    
                         'ZLim',   [ -tmpLim , tmpLim ] , ...
                         'view',   [0, 90], 'fontsize', 30  )      

    scatter( ani.hAxes{ 1 }, data_raw3.p0_arr( 1, 1 ), data_raw3.p0_arr( 2, 1 ), 300, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
    scatter( ani.hAxes{ 1 }, data_raw3.p0_arr( 1, end ), data_raw3.p0_arr( 2, end ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )
    
    plot( [ data_raw3.p0_arr( 1 , 1 ), data_raw3.p0_arr( 1, end ) ], [ data_raw3.p0_arr( 2,1  ),  data_raw3.p0_arr( 2,end )] , 'linewidth', 3, 'linestyle', '--', 'color', c.black )

    ani.adjustFigures( 3 );                     
    a3 = ani.hAxes{ 3 };
    
    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )
    
    plot( a3, data_raw2.t_arr, data_raw2.dq_arr, 'linewidth', 5, 'color', [ 0.2, 0.2, 0.2 ] )
    set( a3, 'fontsize', 30, 'xlim', [0, 3 ], 'ylim', [ -1, 1 ] )
    title( 'Dynamic Motor Primitives', 'fontsize', 30 )

    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.dq_arr( 1, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 3, tmp );
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.dq_arr( 2, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 3, tmp );    
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.dq_arr( 3, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 3, tmp );        
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.dq_arr( 4, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 3, tmp );            
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.dq_arr( 5, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 3, tmp );                
    
    xlabel( ani.hAxes{ 3 }, 'Time (s)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 3 }, 'Joint Velicity (rad/s)', 'fontsize', 30 )
    

    ani.adjustFigures( 2 );                     
    a2 = ani.hAxes{ 2 };
            
    plot( a2, data_raw2.t_arr, data_raw2.q_arr, 'linewidth', 5, 'color', [ 0.2, 0.2, 0.2 ] )
    set( a2, 'fontsize', 30, 'xlim', [0, 3 ], 'ylim', [ -.4, .1 ] )
%     title( 'Dynamic Motor Primitives', 'fontsize', 30 )

    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.q_arr( 1, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 2, tmp );
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.q_arr( 2, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',  [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 2, tmp );    
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.q_arr( 3, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 2, tmp );        
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.q_arr( 4, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 2, tmp );            
    
    tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.q_arr( 5, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                    'SizeData',  300, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
    ani.addTrackingPlots( 2, tmp );                
    
    xlabel( ani.hAxes{ 2 }, 'Time (s)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 2 }, 'Joint Displacement (rad)', 'fontsize', 30 )

    ani.run( 0.70, 0.0, 3.0, true, strcat( mode, TASK )     )

   
end



%% ==================================================================
%% (--) Sequence of Discrete Movements

clear gObjs data*

file_name1 = '../results/sequence/movement/ctrl_task_dmp.mat';
file_name2 = '../results/sequence/motor/ctrl_task_imp.mat';

data_raw1 = load( file_name1 );
data_raw2 = load( file_name2 );

g_old = data_raw2.p0_arr( :, 1 ) + [ -0.7, 0.7, 0. ]';
g_new = g_old + [ 1.5, 0.5, 0. ]';


TASK = "_SEQUENCE";


if mode == "MOVEMENT"
    

              
elseif mode == "MOTOR"

    data_raw = data_raw2;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    p0 = data_raw.p0_arr;
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    % Shoulder
    jSize = 1400;
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                  
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );      
              
    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;  
                  
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              
              
    gObjs( 4 ) = myMarker( 'XData', data_raw.p0_arr( 1, :), 'YData', data_raw.p0_arr( 2, :), 'ZData', data_raw.p0_arr( 3, :), ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3       );                    

end

ani = myAnimation( dt, gObjs );   

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 


tmpLim = 2.0;
cen = [ 0.0, 1.0 ];
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 )] , ...                  
                     'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 )] , ...    
                     'ZLim',   [            -tmpLim,            tmpLim] , ...
                     'view',   [0, 90] , 'fontsize', 30    )      

xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )                 

scatter( ani.hAxes{ 1 }, g_old( 1 ), g_old( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.3 )
scatter( ani.hAxes{ 1 }, g_new( 1 ), g_new( 2 ), 300, 'square', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'markerfacealpha', 0.8 )
scatter( ani.hAxes{ 1 }, data_raw2.p0_arr( 1, 1 ), data_raw2.p0_arr( 2, 1 ), 300, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )


ani.adjustFigures( 2 );                     
a2 = ani.hAxes{ 2 };
plot( a2, data_raw.t_arr, data_raw2.p_arr( 1, : ), 'linewidth', 4, 'color', [ 0.2, 0.2, 0.2], 'linestyle', '-' )
plot( a2, data_raw.t_arr, data_raw2.p_arr( 2, : ), 'linewidth', 4, 'color', [ 0.2, 0.2, 0.2], 'linestyle', '-.' )
set( a2, 'xlim', [ 0, 2], 'ylim', [-1, 2.5] )

tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw.p_arr( 1, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                'SizeData',  800, 'LineWidth', 5 , 'MarkerEdgeColor', [ 0.2, 0.2, 0.2] ); 
ani.addTrackingPlots( 2, tmp );            

tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw.p_arr( 2, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                'SizeData',  800, 'LineWidth', 5 , 'MarkerEdgeColor', [ 0.2, 0.2, 0.2] ); 
ani.addTrackingPlots( 2, tmp );  

legend( a2, 'X', 'Y' )

xlabel( ani.hAxes{ 2 }, 'Time (s)', 'fontsize', 30 )
ylabel( ani.hAxes{ 2 }, 'X,Y (m)', 'fontsize', 30 )


ani.adjustFigures( 3 );                     
a3 = ani.hAxes{ 3 };
plot( a3, data_raw.t_arr, data_raw2.q_arr( 1, : ), 'linewidth', 4, 'color', [ 0.2, 0.2, 0.2], 'linestyle', '-' )
plot( a3, data_raw.t_arr, data_raw2.q_arr( 2, : ), 'linewidth', 4, 'color', [ 0.2, 0.2, 0.2], 'linestyle', '-.' )
set( a3, 'xlim', [ 0, 2] )

tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw.q_arr( 1, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                'SizeData',  800, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
ani.addTrackingPlots( 3, tmp );            

tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw.q_arr( 2, : ), 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                'SizeData',  800, 'LineWidth', 5 , 'MarkerEdgeColor',   [ 0.2, 0.2, 0.2] ); 
ani.addTrackingPlots( 3, tmp );  

ylabel( ani.hAxes{ 3 }, 'Joint Displacments (rad)', 'fontsize', 30 )
legend( a3, 'Shoulder', 'Elbow' )

ani.run( 0.50, 0.0, 2.0, true, strcat( mode, TASK )     )



%% ==================================================================
%% (--) Task-space Trajectory Tracking --- Rhythmic
%% -- (-A) Dynamic Motor Primitives 

file_name = '../results/task_space_traj_track/dynamic_motor/rhythmic.mat';
data_raw1 = load( file_name );

file_name = '../results/task_space_traj_track/dynamic_movement/rhythmic.mat';
data_raw2 = load( file_name );

clear gObjs 

TASK = "_RHYTHMIC_TASK";


if mode == "MOVEMENT"
    

              
elseif mode == "MOTOR"

    data_raw = data_raw1;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 2 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;
    
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
                           
              
    % Elbow
    xEL = cos( q_abs( : , 1 )' );
    yEL = sin( q_abs( : , 1 )' );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
                 


    % Shoulder
    xEE = xEL + cos( q_abs( : , 2 )' );
    yEE = yEL + sin( q_abs( : , 2 )' );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              

end

ani = myAnimation( dt, gObjs );   

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

centers = data_raw1.centers;

tmpLim = 1.6;
cen = [ 0.0, 1.0 ];
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 )] , ...                  
                     'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 )] , ...    
                     'ZLim',   [ -tmpLim           , tmpLim           ] , ...
                     'view',   [0, 90], 'fontsize', 30     )      
xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )    
plot( ani.hAxes{1}, centers( 1 ) + 0.5 * cos( 0:0.01:2*pi ), centers( 2) + 0.5 * sin( 0:0.01:2*pi ), 'linewidth', 3, 'linestyle', '--', 'color', c.black )
                 

ani.run( 0.50, 0.0, 8.0, true, strcat( mode, TASK )     )



%% ==================================================================
%% (--) Discrete and Rhythmic Movements 

% Dynamic Movement Primitives
file_name1 = '../results/discrete_and_rhythmic/movement/dmp_discrete.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/discrete_and_rhythmic/motor/ctrl_joint_imp.mat';
data_raw2 = load( file_name2 );

clear gObjs 

TASK = "_DISCRETE_AND_RHYTHMIC";


if mode == "MOVEMENT"
    

              
elseif mode == "MOTOR"

    data_raw = data_raw2;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    q0_abs = cumsum( data_raw.q0_arr,1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;    
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                  
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );    
              
    % Elbow
    x0EL = cos( q0_abs( 1, : ) );
    y0EL = sin( q0_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 3 ) = myMarker( 'XData', x0EL, 'YData', y0EL, 'ZData', zEL, ... 
                             'name', "EL0"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3  );                  


    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;

    x0EE = x0EL + cos( q0_abs( 2, : ) );
    y0EE = y0EL + sin( q0_abs( 2, : ) );
    z0EE = zSH;    
              
              
    % End Effector
    gObjs( 4 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
                
              
    % End Effector
    gObjs( 5 ) = myMarker( 'XData', x0EE, 'YData', y0EE, 'ZData', z0EE, ... 
                             'name', "EE0"  , 'SizeData',  jSize      , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3  );                  

                
end

ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '-' ); 
    ani.connectMarkers( 1, [     "SH",     "EL0",     "EE0" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '--', 'LineWidth', 2 ); 
tmpLim = 1.6;
cen = [ 0.5, 1.0 ];
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 )] , ...                  
                     'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 )] , ...    
                     'ZLim',   [            -tmpLim,            tmpLim] , ...
                     'view',   [0, 90] , 'fontsize', 30    )      

xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )                      

ani.adjustFigures( 2 );   
plot( ani.hAxes{ 2 }, data_raw2.t_arr, data_raw2.q0_tmp_arr( 2, : ), 'linewidth', 2, 'color', c.black )
plot( ani.hAxes{ 2 }, data_raw2.t_arr, data_raw2.q_arr( 2, : ), 'linewidth', 4, 'color', c.orange )
set(  ani.hAxes{ 2 }, 'fontsize', 30, 'xlim', [ 0.0, 16.0], 'ylim', [ -1.0, 3.0] )

tmp = myMarker( 'XData', data_raw2.t_arr, 'YData', data_raw2.q_arr( 2,: ) , 'ZData', zeros( 1, length( data_raw2.t_arr ) ), ...
                'SizeData',  700, 'LineWidth', 5 , 'MarkerEdgeColor',  c.orange ); 
ani.addTrackingPlots( 2, tmp );

ani.run( 0.7, 3.0, 8.0, true, strcat( mode, TASK )     )


%% ==================================================================
%% (--) Obstacle Avoidance  


% Dynamic Movement Primitives
file_name1 = '../results/obstacle_avoidance/movement/ctrl_task_dmp.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/obstacle_avoidance/motor_no_obstacle/ctrl_task_imp.mat';
file_name3 = '../results/obstacle_avoidance/motor_no_obstacle/ctrl_task_imp2.mat';
data_raw2 = load( file_name2 );
data_raw3 = load( file_name3 );

clear gObjs 

TASK = "_OBSTACLE";


if mode == "MOVEMENT"
    

              
elseif mode == "MOTOR"

    data_raw = data_raw2;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;        
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 1.0       ); 
                                  
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 1.0       ); 
              

    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.purple, ...
                  'MarkerFaceColor', c.purple, ...
                  'MarkerFaceAlpha', 1.0       ); 
              
              

end

ani = myAnimation( dt, gObjs );   

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

tmpLim = 1.6;


scatter( data_raw2.p0i( 1 ), data_raw2.p0i( 2 ), 200, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
scatter( data_raw2.p0f( 1 ), data_raw2.p0f( 2 ), 200, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black )
% scatter( 0.5 *( data_raw2.p0i( 1 ) + data_raw2.p0f( 1 ) ), 0.5 *( data_raw2.p0i( 2 ) + data_raw2.p0f( 2 ) ) ...
%         , 1000, 'o', 'markerfacecolor', c.grey, 'markeredgecolor', c.black, 'linewidth', 3 )

cen = [ 0.0, 1.0 ];
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 )] , ...                  
                     'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 )] , ...    
                     'ZLim',   [            -tmpLim,            tmpLim] , ...
                     'view',   [0, 90], 'fontsize', 30     )      

                 
xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )                      
                 
ani.run( 0.70, 0.0, 3.5, true, strcat( mode, TASK )     )



%% ==================================================================
%% (--) Unexpected contact


% Dynamic Movement Primitives
file_name1 = '../results/unexpected_contact/movement/without_obstacle/ctrl_task_dmp.mat';
data_raw1 = load( file_name1 );

% Dynamic Motor Primitives
file_name2 = '../results/unexpected_contact/motor/with_obstacle_no_mod/ctrl_task_imp.mat';
% file_name2 = '../results/unexpected_contact/motor/with_obstacle_mod_Lmax_0p5/ctrl_task_imp_modulated.mat';
data_raw2 = load( file_name2 );

clear gObjs 

TASK = "_UNEXPECTED_CONTACT";


if mode == "MOVEMENT"

    data_raw = data_raw1;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_actual_arr, 1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;    
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                  
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );    


    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
              
    % Obstacle
    xOBS = zeros( 1, length( xEE ) );
    yOBS = 1.2 * ones( 1, length( xEE ) );
    zOBS = zeros( 1, length( xEE ) );
    
    t_idx = data_raw.t_arr >= 2 & data_raw.t_arr <= 3;
    xOBS = -cumsum( t_idx ) * 0.001;
    
    gObjs( 4 ) = myMarker( 'XData', xOBS, 'YData', yOBS, 'ZData', zOBS, ... 
                             'name', "OBS"  , 'SizeData',  jSize * 1.6 , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.pink, ...
                  'MarkerFaceColor', c.pink, ...
                  'MarkerFaceAlpha', 1.0       );     
              
    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 1.8;
    cen = [ -0.0, 1.2 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim+cen( 1 ), tmpLim+cen( 1 )] , ...                  
                         'YLim',   [ -tmpLim+cen( 2 ), tmpLim+cen( 2 )] , ...    
                         'ZLim',   [ -tmpLim, tmpLim] , ...
                         'view',   [0, 90], 'fontsize', 30  )                    
               

    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )                      
                 
    ani.run( 0.50, 0.0, 4.0, true, strcat( mode, TASK )     )

              
elseif mode == "MOTOR"
    
   
    data_raw = data_raw2;
    dt = data_raw.t_arr( 3 ) -  data_raw.t_arr( 2 );
    
    % Change to absolute angle
    q_abs = cumsum( data_raw.q_arr, 1 );
    
    xSH = zeros( 1, length( data_raw.t_arr ) );
    ySH = zeros( 1, length( data_raw.t_arr ) );
    zSH = zeros( 1, length( data_raw.t_arr ) );
    
    jSize = 1400;    
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );                  
              
    % Elbow
    xEL = cos( q_abs( 1, : ) );
    yEL = sin( q_abs( 1, : ) );
    zEL = zSH;
    
    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       );    


    % Shoulder
    xEE = xEL + cos( q_abs( 2, : ) );
    yEE = yEL + sin( q_abs( 2, : ) );
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize       , ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.orange, ...
                  'MarkerFaceColor', c.orange, ...
                  'MarkerFaceAlpha', 1.0       ); 
                  
              
    % Obstacle
    xOBS = zeros( 1, length( xEE ) );
    yOBS = 1.2 * ones( 1, length( xEE ) );
    zOBS = zeros( 1, length( xEE ) );
    
    t_idx = data_raw.t_arr >= 2 & data_raw.t_arr <= 3;
    xOBS = -cumsum( t_idx ) * 0.001;
    
    gObjs( 4 ) = myMarker( 'XData', xOBS, 'YData', yOBS, 'ZData', zOBS, ... 
                             'name', "OBS"  , 'SizeData',  jSize * 4.4, ...
                        'LineWidth',   1       , ...
                  'MarkerEdgeColor', c.pink, ...
                  'MarkerFaceColor', c.pink, ...
                  'Marker', 'square',...
                  'MarkerFaceAlpha', 1.0       );        
              
    gObjs( 5 ) = myMarker( 'XData', data_raw2.p0_arr( 1, : ), 'YData', data_raw2.p0_arr( 2,: ), 'ZData', data_raw2.p0_arr( 3,: ), ... 
                             'name', "X0"  , 'SizeData',  300 , ...
                        'LineWidth',   10       , ...
                  'MarkerEdgeColor', c.black, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0       );                      
              
    ani = myAnimation( dt, gObjs );   

    ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' ); 

    tmpLim = 1.8;
    cen = [ -0.0, 1.2 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim+cen( 1 ), tmpLim+cen( 1 )] , ...                  
                         'YLim',   [ -tmpLim+cen( 2 ), tmpLim+cen( 2 )] , ...    
                         'ZLim',   [ -tmpLim, tmpLim] , ...
                         'view',   [0, 90], 'fontsize', 30  )                     

    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )                      
                 
    ani.run( 0.7, 0.5, 3.5, true, strcat( mode, TASK )     )
    
end

