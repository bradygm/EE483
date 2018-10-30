syms mc mr s u jc d fe T F b k m w r m2 m1 x k1 k2 l a
% A = [mc*s^2+2*mr*s^2+u*s,0,fe;
%      0,mc*s^2+2*mr*s^2,0;
%      0,0,jc*s^2+2*mr*d^2*s^2 ];
% A = [0 1 0 0; w^2 0 0 2*w*r;0 0 0 1; 0 (-2*w/r) 0 0];
% B = [0;-1/m;0;0];
% C = [1 0 0 0; 0 0 0 1];
% D = [0;0];
% 
% y = C*(s*eye(4)-A)^-1*B+D
%  invA = inv(A)
%  invA*[0;F;T]
%  
% syms m1 m2 l g z
% B = [s^2*m1, m1*g;
%      m1*g/l,s^2*((m2*l^2+3*m1*z^2)/(l*3))];
%  inv(B)*[0;1];

%test simplifiy
% A = [(s^2-w^2) (-2*w*r*s^2); s^2 (-2*w*s/r) ];
% invA = inv(A)
% invA*[(-1/m); 0]
% 
% sys = tf(1,[1 -1 0 1])
% pzmap(sys)


%TF
% num = [1];
% den = [1 2 4];
% sysTF = tf(num,den);
% stepplot(sysTF)

% zeta = 1/sqrt(2);
% w0 = 4;
% H = tf(w0^2,[1 2*zeta*w0 w0^2])
% stepplot(H)
% 
% stepinfo(H)

% % 
A = [0 1 0 0; w^2 0 0 2*w*r;0 0 0 1; 0 (-2*w/r) 0 0]
%Find eigenvalues
 e = eig(A)

% a = 1/(m2*s^2+b*x+k2-(k2^2*l)/(.25*m1*l^2*s^2+k1*a^2+k2*l))

% sys = tf(12,[1 4 3])
% zero(sys)


