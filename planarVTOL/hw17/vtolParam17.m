% satellite parameter file
addpath ./.. % adds the parent directory to the path
VTOLParam % general satellite parameters

% load parameters from HW 10
addpath ./../hw10
vtolParamHW10


%Altitude Control
figure(1)
Ch_pid = tf([(P.kd_h+P.kp_h*P.sigma),(P.kp_h+P.ki_h*P.sigma),P.ki_h],[P.sigma,1,0])
Palt = tf([1/(2*P.mr+P.mc)],[1,0,0]);
bode(Palt*Ch_pid/(1+Palt*Ch_pid));
hold on
margin(Palt*Ch_pid)
legend('Closed Loop','Open Loop')


%Latitudinal Control
figure(2)
Pt_in = tf([1/(P.Jc+2*P.mr*P.d^2)],[1,0,0]);
Ct_pd = tf([(P.kd_th+P.kp_th*P.sigma),P.kp_th],[P.sigma, 1])
margin(Pt_in*Ct_pd)
hold on
bode(Pt_in*Ct_pd/(1+Pt_in*Ct_pd));


legend('Closed Loop','Open Loop')



figure(3)
Pz_out = tf([-P.f_e/(P.mc+2*P.mr)],[1,P.u/(P.mc+2*P.mr),0]);
Pz_out = Pz_out*(Pt_in*Ct_pd/(1+Pt_in*Ct_pd));
Cz_pd = tf([(P.kd_z+P.sigma*P.kp_z), P.kp_z], [P.sigma, 1]);
Cz_pid = tf([(P.kd_z+P.kp_z*P.sigma),(P.kp_z+P.ki_z*P.sigma),P.ki_z],[P.sigma,1,0])
margin(Pz_out*Cz_pid)
hold on
bode(Pz_out*Cz_pid/(1+Pz_out*Cz_pid))

legend('Closed Loop','Open Loop')



% % transfer functions
% P_in = tf([1/P.Js],[1,P.b/P.Js,P.k/P.Js]);
% P_out = tf([P.b/P.Jp, P.k/P.Jp],[1,P.b/P.Jp,P.k/P.Jp]);
% 
% C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
% C_out = tf([(P.kd_phi+P.kp_phi*P.sigma),(P.kp_phi+P.ki_phi*P.sigma),P.ki_phi],[P.sigma,1,0]);
% 
% % margin and bode plots 
% figure(2), clf, margin(P_in*C_in), grid on, hold on
% bode(P_in*C_in/(1+P_in*C_in)) 
% margin(P_out*C_out)
% bode(P_out*C_out/(1+P_out*C_out))
% legend('Open Loop-Inner', 'Closed Loop-Inner','Open Loop-Outer', 'Closed Loop-Outer')