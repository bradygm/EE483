% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
springParam % general arm parameters

%  tuning parameters
%tr = 0.8; % part (a)
tr = 3.5;
% tr = 1.8;

zeta = .707;

% equalibrium position and force
P.z_e = 0;
P.f_e = P.k*P.z_e;

% desired closed loop polynomial
wn = 2.2/tr; %For the case of zeta = .707
Delta_cl_d = [1, 2*zeta*wn, wn^2];
des_poles = roots(Delta_cl_d);

%-----------
% state space design
A = [0 1; (-P.k/P.m) (-P.b/P.m)];
B = [0; (1/P.m)];
C = [1 0];


% Test for controllable or not
if det(ctrb(A,B))==0 
    disp('System Not Controllable'); 
else
    P.K = place(A,B,des_poles); 
    P.kr = -1/(C*inv(A-B*P.K)*B);
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t kr: %f\n', P.kr)



