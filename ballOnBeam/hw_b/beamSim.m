beamParamHWA  % load parameters

% instantiate reference input classes 
beam = ballBeamDynamics(P);
reference = signalGenerator(0.5, 0.1); 
forceRef = signalGenerator(10.8, 0.01);


% instantiate the data plots and animation
dataPlot = plotDataBeam(P);
animation = beamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    t_next_plot = t + P.t_plot;  % advance time by t_plot
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f = forceRef.square(t);  % Calculate the input force
        beam.propagateDynamics(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    animation.drawBeamBall(beam.state);
    dataPlot.updatePlots(t, r, beam.state, f);    
    pause(0.1)
end


