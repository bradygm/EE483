% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
springParam % general arm parameters

%  tuning parameters
%tr = 0.8; % part (a)
% tr = 2.2;
tr = 2.2;

zeta = 0.907;

% equalibrium angle and torque
P.z_e = 0;
P.f_e = P.k*P.z_e;

% desired closed loop polynomial
wn = 2.2/tr; %For the case of zeta = .707
Delta_cl_d = [1, 2*zeta*wn, wn^2];


P.kp = (Delta_cl_d(3)-.6)/.2; %Can swap out .6 for the parameters
P.kd = (Delta_cl_d(2)-.1)/.2;
P.ki = 1; %Integrator gain

fprintf('\t kp: %f\n', P.kp)
fprintf('\t ki: %f\n', P.ki)
fprintf('\t kd: %f\n', P.kd)



