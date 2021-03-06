% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
springParam % general arm parameters

%  tuning parameters
%tr = 0.8; % part (a)
tr = 0.4;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;

% equalibrium angle and torque
P.z_e = 0;
P.f_e = P.k*P.z_e;

% PD gains
p1 = -1;
p2 = -1.5;

P.kp = (-p1*-p2-.6)/.2;
P.kd = (-p1-p2-.1)/.2;

fprintf('\t kp: %f\n', P.kp)
fprintf('\t kd: %f\n', P.kd)



