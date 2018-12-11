sigma =0.1000;
ki = 0.5000;
kp = 3.8720;
kd = 5.7216;

P = tf([.2],[1 0.1 .6])
C_pid = tf([(kd+kp*sigma),(kp+ki*sigma),ki],[sigma,1,0])
figure(1)
bode(P)
hold on
bode(series(P,C_pid))
margin(series(P,C_pid))
% bode(tf([1],[1 0]))
legend('Plant','PID')
