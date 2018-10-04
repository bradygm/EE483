springParamHWA  % load parameters

% instantiate reference input classes 
zRef = signalGenerator(1.5, 0.5); 
rRef = signalGenerator(1, 0.5); 
fRef = signalGenerator(.5,.4);

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
    z = zRef.sin(t);
    r = rRef.square(t);
    f = fRef.square(t);
    % update animation and data plot
    state = [z; 0.0];
    animation.drawSpring(state);
    dataPlot.updatePlots(t, r, state, f);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end