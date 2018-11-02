% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
VTOLParam % general arm parameters

% equalibrium force
% P.z_e = 0;
P.f_e = P.mc*P.g+2*P.mr*P.g;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tuning Parameters for Longitudinal PD Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tr_h = 2;%7; %Rise time for height (longitudinal control)
zeta_h = .707;

% parameters of the open loop transfer function
b0_h = 1/(P.mc+2*P.mr);
a1_h = 0.0;
a0_h = 0.0;

% coefficients for desired inner loop
% Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
wn_h = 2.2/tr_h;     % Natural frequency
alpha1_h = 2.0*zeta_h*wn_h;
alpha0_h = wn_h^2;

% compute gains
% Delta(s) = s^2 + (a1 + b0*kd)*s + (a0 + b0*kp)
P.kp_h = (alpha0_h-a0_h)/b0_h;
P.ki_h = .2; % integral gain for height
P.ki_hlimit = .3;
P.kd_h = (alpha1_h-a1_h)/b0_h;
DC_gain_h = P.kp_h/P.kp_h;

fprintf('\t kp_h: %f\n', P.kp_h)
fprintf('\t ki_h: %f\n', P.ki_h)
fprintf('\t kd_h: %f\n', P.kd_h)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PD Control: Time Design Strategy For Lateral Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


tr_th = .3;%.8;          % Rise time for inner loop (theta)
zeta_th = 0.707;       % Damping Coefficient for inner loop (theta)
M = 10.0;%10.0;              % Time scale separation between inner and outer loop
zeta_z = 0.707;        % Damping Coefficient fop outer loop (z)


% saturation limits
P.error_max = 1;        		  % Max step size,m
P.theta_max = 90.0*pi/180.0;  % Max theta, rads

%---------------------------------------------------
%                    Inner Loop
%---------------------------------------------------
% parameters of the open loop transfer function
b0_th = 1/(2*P.mr*P.d^2+P.Jc);
a1_th = 0.0;
a0_th = 0.0;

% coefficients for desired inner loop
% Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
wn_th = 2.2/tr_th;     % Natural frequency
alpha1_th = 2.0*zeta_th*wn_th;
alpha0_th = wn_th^2;

% compute gains
% Delta(s) = s^2 + (a1 + b0*kd)*s + (a0 + b0*kp)
P.kp_th = (alpha0_th-a0_th)/b0_th;
P.kd_th = (alpha1_th-a1_th)/b0_th;
DC_gain = P.kp_th/P.kp_th;

%---------------------------------------------------
%                    Outer Loop
%---------------------------------------------------
% parameters of the open loop transfer function
b0_z = -(P.f_e/(P.mc+2*P.mr));
a1_z = P.u/(P.mc+2*P.mr);
a0_z = 0;

% coefficients for desired outer loop
% Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
tr_z = M*tr_th;  % desired rise time, s
wn_z = 2.2/tr_z;  % desired natural frequency
alpha1_z = 2.0*zeta_z*wn_z;
alpha0_z = wn_z^2;

% compute gains
% Delta(s) = s^2 + (a1 + b0*kd*DC_gain)*s + (a0 + b0*kp*DC_gain)
P.kp_z = (alpha0_z-a0_z)/(DC_gain*b0_z);
P.ki_zlimit = 0.01;
P.ki_z = 0.01; % integral gain for outer loop
P.kd_z = (alpha1_z-a1_z)/(DC_gain*b0_z);

fprintf('\t DC_gain: %f\n', DC_gain)
fprintf('\t kp_th: %f\n', P.kp_th)
fprintf('\t kd_th: %f\n', P.kd_th)
fprintf('\t kp_z: %f\n', P.kp_z)
fprintf('\t ki_z: %f\n', P.ki_z)
fprintf('\t kd_z: %f\n', P.kd_z)



