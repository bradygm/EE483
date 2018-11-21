% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
springParam % general arm parameters

%  tuning parameters
%tr = 0.8; % part (a)
% tr = 3.5;
tr = 2.2;
integrator_pole = -3;
zeta = .707;

wn_obs = 10;  % natural frequency for observer
zeta_obs = 0.707; % damping ratio for observer

% equalibrium position and force
P.z_e = 0;
P.f_e = P.k*P.z_e;

% desired closed loop polynomial
wn = 2.2/tr; %For the case of zeta = .707
des_char_poly = conv([1,2*zeta*wn,wn^2],poly(integrator_pole));
des_poles = roots(des_char_poly);

%-----------
% state space design
A = [0 1; (-P.k/P.m) (-P.b/P.m)];
B = [0; (1/P.m)];
C = [1 0];
P.C = C;
P.A = A;
P.B = B;

A1 = [A, zeros(2,1); -C, 0];
B1 = [B; 0];

% Test for controllable or not
if det(ctrb(A,B))==0 
    disp('System Not Controllable'); 
else
    K1 = place(A1,B1,des_poles); 
    P.K = K1(1:2);
    P.ki = K1(3);
end

%-------------
%observer design 

% desired observer poles
des_obsv_char_poly = [1,2*zeta_obs*wn_obs,wn_obs^2];
des_obsv_poles = roots(des_obsv_char_poly);

%Check observability
if det(obsv(A,C))==0
    disp('System Not Observable'); 
else 
    P.L = place(A',C',des_obsv_poles)'; 
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L^T: [%f, %f]\n', P.L(1), P.L(2))



