springParamHWA  % load parameters

% instantiate reference input classes 
spring = springDynamics(P);
reference = signalGenerator(1, 0.5); 
forceRef = signalGenerator(2.2,.2);

movieBool = false;
image_array = [];
iteration = 1;
% instantiate the data plots and animation
dataPlot = plotDataSpring(P);
animation = springAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    % update animation and data plot
    t_next_plot = t + P.t_plot;  % advance time by t_plot
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f = forceRef.square(t);  % Calculate the input force
        spring.propagateDynamics(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    animation.drawSpring(spring.state);
    dataPlot.updatePlots(t, r, spring.state, f);

    pause(0.1)
end