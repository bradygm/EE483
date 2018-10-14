vtolParamHW7;  % load parameters

% instantiate spring, controller, and reference input classes 
% Instantiate Dynamics class
vtol = vtolDynamics(P);
ctrl = vtolController(P);  
amplitude = 1; % amplitude of reference input
offset = 0;
frequency = 0.01; % frequency of reference input
reference = signalGenerator(amplitude, frequency, offset); 


% instantiate the data plots and animation
dataPlot = plotVTOLData(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, vtol.outputs());  % Calculate the control value
        vtol.propagateDynamics(u/2,u/2);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots   
    animation.drawVTOL(vtol.state,u);
    dataPlot.updatePlots(t, ref_input,1, vtol.state, u/2,u/2);
end


