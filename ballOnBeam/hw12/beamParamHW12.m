% inverted pendulum - parameter file for hw8
addpath ./.. % adds the parent directory to the path
beamParam % general pendulum parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      State Space Pole Placement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_th = .15;%.35;          % Rise time for inner loop (theta)
zeta_th = 0.707;       % Damping Coefficient for inner loop (theta)
wn_th = 2.2/tr_th;     % Natural frequency
M = 10.0;              % Time scale separation between inner and outer loop

tr_z = M*tr_th;  % desired rise time, s
zeta_z = 0.707;        % Damping Coefficient fop outer loop (z)
wn_z = 2.2/tr_z;  % desired natural frequency
integrator_pole = -3; % integrator pole

a = P.m2*P.ell^2/3+P.m1*P.z0^2;
A = [0 0 1 0;
     0 0 0 1;
     0 -P.g 0 0;
     (-P.m1*P.g/a) 0 0 0];
B = [0; 0; 0; P.ell/a];
C = [1 0 0 0;0 1 0 0];
Cout = C(1,:);

A1 = [A, zeros(4,1); -Cout, 0];
B1 = [B; 0];

des_char_poly = conv(conv([1, 2*zeta_z*wn_z, wn_z^2], [1, 2*zeta_th*wn_th, wn_th^2]),poly(integrator_pole));
des_poles = roots(des_char_poly);

% Compute the gains if the system is controllable
if det(ctrb(A, B)) == 0
    disp('The system is not controllable')
else
    K1 = place(A1, B1, des_poles);
    P.K  = K1(1:4);
    P.ki = K1(5);
end

sprintf('K: (%f, %f, %f, %f)\nki: %f\n', P.K(1), P.K(2), P.K(3), P.K(4), P.ki)

% saturation limits
P.error_max = 1;        		  % Max step size,m
P.theta_max = 90.0*pi/180.0;  % Max theta, rads





