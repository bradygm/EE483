VTOLParamHWA  % load parameters

% instantiate reference input classes 
vtol = vtolDynamics(P);
reference = signalGenerator(0.5, 0.1);
forceRref = signalGenerator(9.1, .01);
forceLref = signalGenerator(9, .01);

% instantiate the data plots and animation
dataPlot = plotVTOLData(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.sin(t);
    t_next_plot = t + P.t_plot;  % advance time by t_plot
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        fr = forceRref.square(t);  % Calculate the input force
        fl = forceLref.square(t);  % Calculate the input force
        vtol.propagateDynamics(fl,fr);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plot  
    animation.drawVTOL(vtol.state,r);
    dataPlot.updatePlots(t, r, vtol.state, fl, fr);
    pause(0.1)
end


