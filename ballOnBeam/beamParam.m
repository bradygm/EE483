% Inverted Pendulum Parameter File

% Physical parameters of the inverted pendulum known to the controller
P.m1 = 0.35;     % Mass of the ball, kg
P.m2 = 2.0;      % Mass of the beam, kg
P.ell = 0.5;    % Length of the rod, m
P.g = 9.8;       % Gravity, m/s**2

% parameters for animation
P.w = 0.02;       % Width of the beam, m
P.radius = 0.06; % Radius of circular part of pendulum

% initial conditions
P.z0 = P.ell/2;         % initial position of ball in m
P.zdot0 = 0;      % initial velocity of ball in m/s
P.theta0 = 0;     % initial angle of rod in rad
P.thetadot0 = 0;  % initial angular velocity of rod in rad/sec

% Simulation parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for controller
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% control saturation limits
P.F_max = 15; % Max Force, N
