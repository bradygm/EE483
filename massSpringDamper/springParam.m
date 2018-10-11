% Mass Spring Damper Parameter File

clear all

% Physical parameters of arm known to the controller
P.m = 5;  % kg
P.k = 3; % N/m
P.b = 0.5; % N m s
P.g = 9.8; % m/s^2

% parameters for animation
P.w = 2;       % Width of the cart, m
P.h = 2;      % Height of the cart, m
P.gap = 2;   % Gap between the cart and y-axis

% initial conditions
P.z0 = 0;     % initial angle of the arm in rad
P.zdot0 = 0;  % initial angular rate in rad/sec

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% % dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% % % control saturation limits
P.f_max = 2; % max force