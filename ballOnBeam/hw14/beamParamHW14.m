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
integrator_pole = -1; % integrator pole

% tuning parameters for observer
tr_z_obs = tr_z/7; %7
tr_theta_obs = tr_th/5; %7
dist_obsv_pole = -3; %-4;   % pole for disturbance observer

a = P.m2*P.ell^2/3+P.m1*P.z0^2;
A = [0 0 1 0;
     0 0 0 1;
     0 -P.g 0 0;
     (-P.m1*P.g/a) 0 0 0];
B = [0; 0; 0; P.ell/a];
C = [1 0 0 0;0 1 0 0];
P.A = A;
P.B = B;
P.C = C;
Cout = C(1,:);

A1 = [A, zeros(4,1); -Cout, 0];
B1 = [B; 0];

des_char_poly = conv(conv([1, 2*zeta_z*wn_z, wn_z^2], [1, 2*zeta_th*wn_th, wn_th^2]),poly(integrator_pole));
des_poles = roots(des_char_poly);

% Compute the gains if the system is controllable
if det(ctrb(A1, B1)) == 0
    disp('The system is not controllable')
else
    K1 = place(A1, B1, des_poles);
    P.K  = K1(1:4);
    P.ki = K1(5);
end

% observer design
A2 = [P.A, P.B; zeros(1,4), zeros(1,1)];
C2 = [P.C, zeros(2,1)];

wn_z_obs = 2.2/tr_z_obs; % natural frequency for position
wn_th_obs = 2.2/tr_theta_obs; % natural frequency for angle
des_obsv_char_poly = conv([1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
                      [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(A2,C2))~=5
    disp('System Not Observable'); 
else % if so, compute gains
    L2 = place(A2', C2', [des_obsv_poles;dist_obsv_pole])';
    P.L = L2(1:4,:);
    P.Ld = L2(5,:);
end


sprintf('K:\t[%f, %f, %f, %f]\nki:\t%f\nL^T:\t[%f, %f, %f, %f;\n\t %f, %f, %f, %f]\nLd:\t[%f, %f]',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki,...
    P.L(1,1), P.L(2,1), P.L(3,1), P.L(4,1), P.L(1,2), P.L(2,2), P.L(3,2), P.L(4,2),...
    P.Ld(1), P.Ld(2))

% saturation limits
P.error_max = 1;        		  % Max step size,m
P.theta_max = 90.0*pi/180.0;  % Max theta, rads





