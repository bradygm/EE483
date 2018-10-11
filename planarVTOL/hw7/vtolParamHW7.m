% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
VTOLParam % general arm parameters

%  tuning parameters
%tr = 0.8; % part (a)
tr = 0.4;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;

% equalibrium angle and torque
% P.z_e = 0;
P.f_e = P.mc*P.g+2*P.mr*P.g;

% PD gains
p1 = -.2;
p2 = -.3;
b_0 = 1/(P.mc+2*P.mr);

P.kp = (-p1*-p2)/b_0;
P.kd = (-p1-p2)/b_0;

fprintf('\t kp: %f\n', P.kp)
fprintf('\t kd: %f\n', P.kd)



