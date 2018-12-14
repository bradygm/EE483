m1 = 0.35;     % Mass of the ball, kg
m2 = 2.0;      % Mass of the beam, kg
ell = 0.5;    % Length of the rod, m
g = 9.8;       % Gravity, m/s**2

sigma = 0.0500;
kp_th = 126.7419;
kd_th = 9.7753;
kp_z = -0.3430;
kd_z = -0.2645;
ki_z = -0.1000;

P = tf([3*ell],[m2*ell^2 + 3*m1*.25,0,0])
C_pd = tf([(kd_th+kp_th*sigma),kp_th],[sigma, 1])
figure(1)
margin(series(P,C_pd))
hold on
margin(P*C_pd/(1+P*C_pd))


% bode(tf([1],[1 0]))
legend('Open Loop','Closed Loop')

Pz = tf([-9.8],[1,0,0])
C_pid = tf([(kd_z+kp_z*sigma),(kp_z+ki_z*sigma),ki_z],[sigma,1,0])
ref = tf([1.2],[1,0,.36])
figure(2)
margin(series(Pz,C_pid))
hold on
margin(Pz*C_pid/(1+Pz*C_pid))
hold on
P = tf([3*ell],[m2*ell^2 + 3*m1*.25,0,0])
C_pd = tf([(kd_th+kp_th*sigma),kp_th],[sigma, 1])
bode(series(P,C_pd))
hold on
bode(P*C_pd/(1+P*C_pd))

% bode(tf([1],[1 0]))
legend('Open Loop','Closed Loop')