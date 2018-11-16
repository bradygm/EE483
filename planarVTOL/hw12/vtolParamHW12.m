% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
VTOLParam % general arm parameters

% equalibrium force
% P.z_e = 0;
P.f_e = P.mc*P.g+2*P.mr*P.g;


tr_h = 1.5;%7; %Rise time for height (longitudinal control)
tr_th = .3;%.8;          % Rise time for inner loop (theta)

M = 10.0;%10.0;              % Time scale separation between inner and outer loop
tr_z = M*tr_th;  % desired rise time, s
zeta_z = 0.707;        % Damping Coefficient fop outer loop (z)
zeta_th = 0.707;       % Damping Coefficient for inner loop (theta)
zeta_h = .707;


wn_h = 2.2/tr_h;     % Natural frequency
wn_th = 2.2/tr_th;     % Natural frequency
wn_z = 2.2/tr_z;  % desired natural frequency

integrator_pole_lon = -.8; % integrator pole
integrator_pole_lat = -.65; % integrator pole

% state space design for h control
A = [0 1;
     0 0];
B = [0;
     (1/(P.mc+2*P.mr))];
C = [1 0];

A1 = [A, zeros(2,1); -C, 0];
B1 = [B; 0];

des_char_poly = conv([1,2*zeta_h*wn_h,wn_h^2],poly(integrator_pole_lon));
des_poles = roots(des_char_poly);
% is the system controllable?
if (det(ctrb(A1,B1))==0) 
    disp('System Not Controllable'); 
else
    K1 = place(A1, B1, des_poles);
    P.K  = K1(1:2);
    P.ki = K1(3);
end



% state space design
A = [0 0 1 0;
     0 0 0 1;
     0 (-P.f_e/(P.mc+2*P.mr)) (-P.u/(P.mc + 2*P.mr)) 0;
     0 0 0 0];
B = [0;
     0;
     0;
     (1/(P.Jc+2*P.mr*P.d^2))];
C = [1 0 0 0;
     0 1 0 0];

A1 = [A, zeros(4,1); -C(1,:), 0];
B1 = [B; 0];

des_char_poly = conv(conv([1,2*zeta_th*wn_th,wn_th^2],...
                [1,2*zeta_z*wn_z,wn_z^2]),poly(integrator_pole_lat));
des_poles = roots(des_char_poly);
% is the system controllable?
if (det(ctrb(A1,B1))==0) %NOT SQUARE!!!!!!!!!
    disp('System Not Controllable'); 
else
    Ktemp = place(A1,B1,des_poles);
    P.K = [P.K(1) 0 0 P.K(2) 0 0; 0 Ktemp(1) Ktemp(2) 0 Ktemp(3) Ktemp(4)];
    P.ki = [P.ki 0; 0 Ktemp(5)];
end






sprintf('K: (%f, %f, %f, %f, %f, %f)\nkr: %f %f\n ', P.K(1), P.K(2), P.K(3), P.K(4), P.K(5), P.K(6), P.ki(1),P.ki(2))






% saturation limits
P.error_max = 1;        		  % Max step size,m
P.theta_max = 90.0*pi/180.0;  % Max theta, rads



