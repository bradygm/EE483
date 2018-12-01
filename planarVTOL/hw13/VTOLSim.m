vtolParamHW13;  % load parameters

% instantiate spring, controller, and reference input classes 
% Instantiate Dynamics class
vtol = vtolDynamics(P);
ctrl = vtolController(P);  
amplitude = .5; % amplitude of Height reference input
offset = 1;
frequency = 0.03; % frequency of reference input
referenceH = signalGenerator(amplitude, frequency, offset); 
amplitude = 2.5; % amplitude of target reference input
offset = 3;
frequency = 0.05; % frequency of reference input
referenceZt = signalGenerator(amplitude, frequency, offset); 


% instantiate the data plots and animation
dataPlot = plotVTOLData(P);
animation = VTOLAnimation(P);
observerPlot = plotVTOLObserverData(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = referenceH.square(t);
    ref_zt = referenceZt.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        [Fl,Fr] = ctrl.u(ref_input,ref_zt, vtol.outputs());  % Calculate the control value for F
        vtol.propagateDynamics(Fl,Fr);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots   
    animation.drawVTOL(vtol.state,ref_zt);
    dataPlot.updatePlots(t, ref_input, ref_zt, vtol.state, Fl,Fr);
    observerPlot.updatePlots(t, vtol.states, ctrl.x_hat);
end
observerPlot.error();


