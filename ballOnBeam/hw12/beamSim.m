beamParamHW12;  % load parameters


% instantiate beam, controller, and reference input classes 
% Instantiate Dynamics class
beam = ballBeamDynamics(P); 
ctrl = beamController(P);  
amplitude = 0.15; % amplitude of reference input
offset = 0.25;
frequency = 0.05; % frequency of reference input
reference = signalGenerator(amplitude, frequency,offset);  

% set disturbance input
disturbance = 0.5;

% instantiate the data plots and animation
dataPlot = plotDataBeam(P);
animation = beamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, beam.outputs());  % Calculate the control value
        beam.propagateDynamics(u+disturbance);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawBeamBall(beam.states);
    dataPlot.updatePlots(t, ref_input, beam.state, u);
end


