% [Project]        DMP Basics
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
%% (--) Analyze the Dynamic Movement Primitives 
%% -- (-A) Discrete Movement Example

file_name = '../results/example/movement_primitives/discrete_example.mat';
data_raw = load( file_name );

t_arr   = data_raw.t_arr;
tau     = data_raw.tau;
alpha_s = data_raw.alpha_s;

sgtitle( 'Discrete Movement', 'fontsize', 30 )

% Plot the Canonical System

subplot( 2, 2, 1 )
hold on
plot( t_arr, exp( -t_arr * alpha_s / tau ), 'linewidth', 5, 'color', c.black );
xlabel( '$t$', 'fontsize', 30)
ylabel( '$s$', 'fontsize', 30 )
set( gca, 'xlim', [0, 1], 'ylim', [0,1] )

text( -0.15,0.9,'A')

% Plot the Y Values 
subplot( 2, 2, 4 )
hold on
t_des = data_raw.t_des;
y_arr = data_raw.y_arr;
y_des = data_raw.y_des;
plot( t_arr, y_arr, 'linewidth', 3, 'color', c.black );
plot( t_des, y_des, 'linewidth', 4, 'linestyle', '--', 'color', c.pink );
xlabel( '$t$', 'fontsize', 30)
ylabel( '$y, ~y_{des}$', 'fontsize', 30 )
legend( { '$y$', '$~y_{des}$' }, 'fontsize', 30, 'location', 'northwest' )
set( gca, 'xlim', [0, 1], 'ylim', [0,1] )
text( -0.15,0.9,'D')

% Plot the Gaussian Basis Functions.
subplot( 2, 2, 2 )
hold on
text( 0.28,0.9,'B')

centers = data_raw.centers;
heights = data_raw.heights;

for i = 1 : length( centers )
   ci = centers( i );
   hi = heights( i );
   s_arr = exp( -t_arr * alpha_s / tau );
   plot( s_arr, myGaussian( s_arr, ci, hi ), 'linewidth', 3 );
end

set(gca, 'xlim', [ min( s_arr ), max( s_arr ) ], 'fontsize', 25 );
xlabel( '$s$', 'fontsize', 30)
ylabel( '$\phi(s)$', 'fontsize', 30 )


% Plot the Nonlinear Forces
subplot( 2, 2, 3 )
hold on
text( 0.28,7.0,'C')


weights = data_raw.weights;

f_arr = zeros( 1, length( s_arr ) );
for i = 1 : length( s_arr )
   tmp_arr = zeros( 1, length( weights ) );
   for j = 1 : length( weights )
        tmp_arr( j ) = myGaussian( s_arr( i ), centers( j ), heights( j ) );
   end
   f_arr( i ) = sum( tmp_arr.*weights ) / sum( tmp_arr );
end

plot( s_arr, f_arr, 'linewidth', 5, 'color', c.black );
set(gca, 'xlim', [ min( s_arr ), max( s_arr ) ], 'fontsize', 25 );
xlabel( '$s$', 'fontsize', 30)
ylabel( '$f^*(s)$', 'fontsize', 30 )

mySaveFig( gcf, 'movement_discrete_example' )

%% -- (-B) Rhythmic Movement Example

file_name = '../results/example/movement_primitives/rhythmic_example.mat';
data_raw = load( file_name );

t_arr   = data_raw.t_arr;
tau     = data_raw.tau;

sgtitle( 'Rhythmic Movement', 'fontsize', 30 )

% Plot the Canonical System
subplot( 2, 2, 1 )
hold on
text( -2.45,5.5,'A')
s_arr = mod( t_arr/tau, 2 * pi );
plot( t_arr, s_arr, 'linewidth', 5, 'color', c.black );
xlabel( '$t$', 'fontsize', 30)
ylabel( '$s$', 'fontsize', 30 )
set( gca, 'xlim', [ 0, 6*pi ],'ylim', [ 0, 2*pi ], 'xtick', [ 0, 2*pi, 4*pi, 6*pi ], 'ytick', [ 0, pi, 2*pi],  'yticklabel', { '0', '$\pi$', '$2\pi$' }, 'xticklabel', { '0', '$2\pi$', '$4\pi$', '$6\pi$' }, 'fontsize', 25 )

% Plot y and y_des
subplot( 2, 2, 4 )
hold on
text( -2.5,0.83,'D')
omega = 1 / tau;

plot( t_arr, data_raw.y_arr, 'linewidth', 3, 'color', c.black );
plot( t_arr, sin( omega * t_arr ), 'linewidth', 4, 'linestyle', '--', 'color', c.pink );
set( gca, 'xlim', [ 0, max( t_arr ) ] )
xlabel( '$t$', 'fontsize', 30)
ylabel( '$y, ~y_{des}$', 'fontsize', 30 )
set( gca, 'xlim', [ 0, 6*pi ], 'xtick', [ 0, 2*pi, 4*pi, 6*pi ], 'xticklabel', { '0', '$2\pi$', '$4\pi$', '$6\pi$' }, 'fontsize', 25 )
legend( { '$y$', '$~y_{des}$' }, 'fontsize', 20, 'location', 'southwest' )

% Plot the Gaussian Basis Functions.
subplot( 2, 2, 2 )
hold on
text( -0.8,0.90,'B')
centers = data_raw.centers;
heights = data_raw.heights;

for i = 1 : length( centers )
   ci = centers( i );
   hi = heights( i );
   idx = find( abs( ( s_arr - 2 * pi ) ) == min( abs( ( s_arr - 2 * pi ) ) ) ) ;
   plot( s_arr( 1 : idx ), myVonMises( s_arr( 1 : idx ), ci, hi ), 'linewidth', 3 );
end

set(gca, 'xlim', [ min( s_arr ), max( s_arr ) ] );
xlabel( '$s$', 'fontsize', 30)
ylabel( '$\phi(s)$', 'fontsize', 30 )
set( gca, 'xlim', [ 0, 2*pi ], 'xtick', [ 0, 1*pi, 2*pi ], 'xticklabel', { '0', '$\pi$', '$2\pi$' }, 'fontsize', 25 )


% Plot the Nonlinear Forces
subplot( 2, 2, 3 )
hold on
text( -0.9,19,'C')

weights = data_raw.weights;

f_arr = zeros( 1, length( s_arr ) );
for i = 1 : idx
   tmp_arr = zeros( 1, length( weights ) );
   for j = 1 : length( weights )
        tmp_arr( j ) = myVonMises( s_arr( i ), centers( j ), heights( j ) );
   end
   f_arr( i ) = sum( tmp_arr.*weights ) / sum( tmp_arr );
end

plot( s_arr( 1: idx), f_arr( 1: idx), 'linewidth', 5, 'color', c.black);
set( gca, 'xlim', [ 0, 2*pi ], 'xtick', [ 0, 1*pi, 2*pi ], 'xticklabel', { '0', '$\pi$', '$2\pi$' }, 'fontsize', 25 )
xlabel( '$s$', 'fontsize', 30)
ylabel( '$f^*(s)$', 'fontsize', 30 )

mySaveFig( gcf, 'movement_rhythmic_example' )

%% ==================================================================
%% (--) Analyze Dynamic Motor Primitives
%% -- (-A) Discrete Movement Example
close all
% Discrete 
file_name_D = '../results/example/motor_primitives/discrete_example.mat';
data_raw_D  = load( file_name_D );

% Rhythmic
file_name_R = '../results/example/motor_primitives/rhythmic_example.mat';
data_raw_R  = load( file_name_R );


subplot( 3, 2, 1 )
hold on
plot( data_raw_D.t_arr, data_raw_D.dq0_arr, 'linewidth', 5, 'color', c.black )
set( gca, 'xlim', [ 0, 1 ], 'xtick', [ 0, 0.5, 1.0], 'fontsize', 25 )
ylabel( '$\dot{x}_0$', 'fontsize', 30 )
title( 'Submovement', 'fontsize', 30 )

subplot( 3, 2, 3 )
hold on
plot( data_raw_D.t_arr, data_raw_D.q_arr,  'linewidth', 4, 'color', c.black );
plot( data_raw_D.t_arr, data_raw_D.q0_arr, 'linewidth', 3, 'color', c.pink, 'linestyle', '--' );
ylabel( '$x$, $x_0$', 'fontsize', 30 )
legend( { '$x$', '$x_0$' }, 'fontsize', 20, 'location', 'northwest' )
set( gca, 'xlim', [ 0, 1 ], 'xtick', [ 0, 0.5, 1.0], 'fontsize', 25 )

subplot( 3, 2, 5 )
hold on
plot( data_raw_D.t_arr, data_raw_D.tau_arr, 'linewidth', 5, 'color', c.black )
ylabel( '$F$', 'fontsize', 30 )
xlabel( '$t$', 'fontsize', 30 )
set( gca, 'xlim', [ 0, 1 ], 'xtick', [ 0, 0.5, 1.0], 'fontsize', 25 )

subplot( 3, 2, 2 )
hold on
plot( data_raw_R.t_arr, data_raw_R.q0_arr, 'linewidth', 5, 'color', c.black )
set( gca, 'xlim', [ 0, 6 * pi ], 'xtick', [ 0, 1*pi, 2*pi, 3*pi, 4*pi, 5*pi, 6*pi ], ...
                            'xticklabel', { '0', '$\pi$', '$2\pi$', '$3\pi$', '$4\pi$' '$5\pi$', '$6\pi$' }, 'fontsize', 25 )
ylabel( '$x_0$', 'fontsize', 30 )
title( 'Oscillation', 'fontsize', 30 )

subplot( 3, 2, 4 )
hold on
plot( data_raw_R.t_arr, data_raw_R.q_arr,  'linewidth', 4, 'color', c.black );
plot( data_raw_R.t_arr, data_raw_R.q0_arr, 'linewidth', 3, 'color', c.pink, 'linestyle', '--' );
set( gca, 'xlim', [ 0, 6 * pi ], 'xtick', [ 0, 1*pi, 2*pi, 3*pi, 4*pi, 5*pi, 6*pi ], ...
                            'xticklabel', { '0', '$\pi$', '$2\pi$', '$3\pi$', '$4\pi$' '$5\pi$', '$6\pi$' }, 'fontsize', 25 )
legend( { '$x$', '$x_0$' }, 'fontsize', 20, 'location', 'northeast' )
ylabel( '$x$, $x_0$', 'fontsize', 30 )

subplot( 3, 2, 6 )
hold on
plot( data_raw_R.t_arr, data_raw_R.tau_arr, 'linewidth', 5, 'color', c.black )
set( gca, 'xlim', [ 0, 6 * pi ], 'xtick', [ 0, 1*pi, 2*pi, 3*pi, 4*pi, 5*pi, 6*pi ], ...
                            'xticklabel', { '0', '$\pi$', '$2\pi$', '$3\pi$', '$4\pi$' '$5\pi$', '$6\pi$' }, 'fontsize', 25 )
ylabel( '$F$', 'fontsize', 30 )
xlabel( '$t$', 'fontsize', 30 )
mySaveFig( gcf, 'motor_example' )

%% ==================================================================
%% (--) Submovements Examples

close all


xi = 0;
xf = 1;
D  = 1;
ti = 0;
t_arr = ti: 0.001: 3;

[y_arr1, dy_arr1, ~] = min_jerk_traj( t_arr, xi, xf, D, ti );
[y_arr2, dy_arr2, ~] = min_jerk_traj( t_arr, xi, -0.6, D, 0.3 );
[y_arr3, dy_arr3, ~] = min_jerk_traj( t_arr, xi,  0.4, D, 0.7 );

subplot( 2, 2, 1 )
hold on
plot( t_arr, 8/15 * dy_arr1, 'linewidth', 4, 'color' ,'k' )
set( gca, 'xlim', [ 0, 1 ], 'fontsize', 25, 'ytick', [0, 1]  )
ylabel( '$\hat{\sigma}(t)$', 'fontsize', 30 )
text( -0.10, 0.90, 'A', 'fontsize', 30 )

subplot( 2, 2, 3 )
hold on

plot( t_arr, dy_arr1, 'linewidth', 4, 'linestyle', '--', 'color', 'k' )
plot( t_arr, dy_arr2, 'linewidth', 4, 'linestyle', '--', 'color', 'k' )
plot( t_arr, dy_arr3, 'linewidth', 4, 'linestyle', '--', 'color', 'k' )
plot( t_arr, ( dy_arr1 + dy_arr2 + dy_arr3 ), 'linewidth', 10, 'color', c.blue )
ylabel( '$\dot{x}_0(t)$', 'fontsize', 30 )
set( gca, 'xlim', [ 0, 1.8], 'ylim', [-1.5,2.0],'xtick', 0:0.6:1.8, 'fontsize', 25 )
text( -0.18, 1.70, 'C', 'fontsize', 30 )

subplot( 2, 2, 2 )
hold on
plot( t_arr, 8/15 * y_arr1, 'linewidth', 3, 'color', 'k' )
set( gca, 'xlim', [ 0, 1],  'fontsize', 25 )
ylabel( '$x_0(t)$', 'fontsize', 30 )

text( -0.10, 0.54, 'B', 'fontsize', 30 )


subplot( 2, 2, 4 )
hold on

plot( t_arr, y_arr1 + y_arr2 + y_arr3, 'linewidth', 10, 'color', c.blue )
ylabel( '$x_0(t)$', 'fontsize', 30 )
% xlabel( '$t$', 'fontsize', 30 )
set( gca, 'xlim', [ 0, 1.8], 'xtick', 0:0.6:1.8, 'fontsize', 25 )
sgtitle( 'Submovements', 'fontsize', 35 )
text( -0.20, 0.72, 'D', 'fontsize', 30 )


han=axes( gcf,'visible','off'); 
han.XLabel.Visible='on';

xlabel(han,'$t$', 'fontsize', 30 );


%% ==================================================================
%% (--) Submovements and Oscillations Examples

close all

xi = 0;
xf = 1;
D  = 1;
ti = 0;
t_arr = ti: 0.001: 1;

[y_arr1, dy_arr1, ~] = min_jerk_traj( t_arr, xi, xf, D, ti );
y_arr1 = y_arr1 * 8/15;
y_arr2 = 0.2 * sin( 12 * pi * t_arr );
subplot( 2, 2, 1 )
hold on
plot( t_arr, y_arr1, 'linewidth', 4, 'color' ,'k' )
set( gca, 'fontsize', 25, 'xtick', 0:0.5:2  )
title( 'Submovement', 'fontsize', 30 )
ylabel( '$x_0(t)$', 'fontsize', 30 )

text( -0.15, 0.55, 'A', 'fontsize', 30 )

subplot( 2, 2, 2 )
hold on
plot( t_arr, y_arr2 , 'linewidth', 4, 'color' ,'k' )
set( gca, 'fontsize', 25, 'xtick', 0:0.5:2  )
    title( 'Oscillation', 'fontsize', 30 )
text( -0.15, 0.165, 'B', 'fontsize', 30 )


subplot( 2, 2, [3,4] )
hold on
plot( t_arr, y_arr1 +  y_arr2 , 'linewidth', 4, 'color' ,'k'  )
plot( t_arr, y_arr1 , 'linewidth', 3, 'linestyle', '--', 'color' ,'k' )
set( gca, 'fontsize', 25, 'xtick', 0:0.5:2  )
title( 'Submovement + Oscillation', 'fontsize', 30 )
xlabel( '$t$', 'fontsize', 30 )
ylabel( '$x_0(t)$', 'fontsize', 30 )
text( -0.07, 0.7, 'C', 'fontsize', 30 )



