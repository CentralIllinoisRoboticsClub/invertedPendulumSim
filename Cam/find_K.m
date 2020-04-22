%Discretized linearized state matrices

Ad = [1.0000    0.0100   -0.0001   -0.0000;
         0    1.0000   -0.0245   -0.0001;
         0         0    1.0036    0.0100;
         0         0    0.7215    1.0036];

Bd = [0.0002;
      0.0313;
      -0.0009;
      -0.1840];
  
Cd = [1     0     0     0;
      0     0     1     0];
  
% desired poles using pole placement
p = [0.9 0.8 0.7 0.95];
K_place = place(Ad, Bd, p)

% try using LQR
R = 1;
Q_diag = [1 1 1 1];
Q = diag(Q_diag);
K_lqr = dlqr(Ad, Bd, Q, R)

