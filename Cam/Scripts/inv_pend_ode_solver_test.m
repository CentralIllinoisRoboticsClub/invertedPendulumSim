clear all, clc

% system parameters
g = 9.8; % gravity
m = 0.08; % kg, pendulum mass, meas 0.08
M = 0.32; % kg, cart mass, meas 0.32
l = 0.17; % meters, center of mass location on pendulum, meas 0.17
I = 3.5e-3; %1/3*m1*r^2 + m2*L^2;, meas 3.5e-3
b = 0.01; % N/(m/s)

% simulation time parameters
Ts = 0.01;
T_total = 10;
tspan = [0:Ts:T_total];

% linearized discretized state space matrices, used for calculating gains
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

% calculate gain matrices
% desired poles using pole placement
p = [0.9 0.8 0.7 0.95];
K_place = place(Ad, Bd, p);

% try using LQR
R = 1;
Q_diag = [1 1 1 1];
Q = diag(Q_diag);
K_lqr = dlqr(Ad, Bd, Q, R);

% ode solver options
opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-5);

K = K_lqr;
X0 = [0 0 0.1 0];

[t, X] = ode45(@(t, X) inv_pend_eqn_motion_test(t, X, K), tspan, X0, opts);

%X(1,k) = X(1,k-1) + X(2,k)*Ts;
%X(3,k) = X(3,k-1) + X(4,k)*Ts;
    
tiledlayout(2,2)
nexttile
plot(t, X(:, 1))
title('position')

nexttile
plot(t, X(:,2))
title('velocity')

nexttile
plot(t, X(:,3))
title('angle')

nexttile
plot(t, X(:,4))
title('angular velocity')