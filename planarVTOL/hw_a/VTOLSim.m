VTOLParamHWA  % load parameters

% instantiate reference input classes 
reference = signalGenerator(0.5, 0.1);
thetaRef = signalGenerator(pi, 0.1);   
zvRef = signalGenerator(1.5, 0.07);
ztRef = signalGenerator(1.5, 0.05);
hRef = signalGenerator(1, 0.5);

% instantiate the data plots and animation
% dataPlot = plotData(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    theta = thetaRef.sin(t);
    zv = zvRef.sin(t)+P.zv0;
    zt = ztRef.sin(t)+P.zt0;
    h = hRef.sin(t)+P.h0;
    % update animation and data plot
    state = [theta; zv; zt; h; 0.0; 0.0];
    animation.drawVTOL(state);
%     dataPlot.updatePlots(t, r, state);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


