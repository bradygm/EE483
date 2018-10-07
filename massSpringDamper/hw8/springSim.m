springParamHW8;  % load parameters

% instantiate spring, controller, and reference input classes 
% Instantiate Dynamics class
spring = springDynamics(P);
ctrl = springController(P);  
amplitude = 1; % amplitude of reference input
frequency = 0.01; % frequency of reference input
reference = signalGenerator(amplitude, frequency); 


% instantiate the data plots and animation
dataPlot = plotDataSpring(P);
animation = springAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, spring.outputs());  % Calculate the control value
        spring.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots   
    animation.drawSpring(spring.state);
    dataPlot.updatePlots(t, ref_input, spring.state, u);
end


