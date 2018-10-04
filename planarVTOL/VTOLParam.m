clear all

% Physical parameters of the satellite known to the controller
P.Jc = .0042; % kg m^2
P.mc = 1;  % kg 
P.mr = .25; % kg
P.ml = .25; % kg
P.d = .3; %m
P.u = .1; %kg/s
P.g = 9.81; %m/s^2


% parameters for animation
P.square = P.d/2.5; %size of center of mass
P.length = .3;  % length of target
P.width = .2; % width of target
P.pw = P.d/6 ; %panel width m
P.pl = P.d/3; %panel length m

% Initial Conditions
P.h0    = 2;
P.zv0      = 2;
P.zt0 = 3;
P.zvdot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;
P.hdot0 = 0;

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

