% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
springParam % general arm parameters

%  tuning parameters
%tr = 0.8; % part (a)
% tr = 3.5;
tr = 2.2;
integrator_pole = -3;
zeta = .707;

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

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)



