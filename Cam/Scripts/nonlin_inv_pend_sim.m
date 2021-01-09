clear all, clc

%% system parameters
g = 9.8; % gravity
m = 0.08; % kg, pendulum mass, meas 0.08
M = 0.32; % kg, cart mass, meas 0.32
l = 0.17; % meters, center of mass location on pendulum, meas 0.17
I = 3.5e-3; %1/3*m1*r^2 + m2*L^2;, meas 3.5e-3
b = 0.5; % friction on cart N/(m/s)
Ts = 0.01; % sample rate

%% linearized continuous state space with damping
Ac = [0 1 0 0;
      0 -b/M -m*g/M 0;
      0 0 0 1;
      0 b/(M*l) (-m*g+M*g)/(M*l) 0];
  
Bc = [0;
      1/M;
      0
      -1/(M*l)];
  
Cc = [1 0 0 0;
      0 0 1 0];

Dc = [0;
      0];

%% discretize linearized state space
sysc = ss(Ac, Bc, Cc, Dc); 
sysd = c2d(sysc, Ts);
[Ad, Bd, Cd, Dd] = ssdata(sysd);

% % linearized discretized state space matrices (no cart friction)
% Ad = [1.0000    0.0100   -0.0001   -0.0000;
%          0    1.0000   -0.0245   -0.0001;
%          0         0    1.0036    0.0100;
%          0         0    0.7215    1.0036];
% 
% Bd = [0.0002;
%       0.0313;
%       -0.0009;
%       -0.1840];
%   
% Cd = [1     0     0     0;
%       0     0     1     0];

%% calculate gain matrices
% find K using pole placement
p = [0.9 0.8 0.7 0.95];
K_place = place(Ad, Bd, p);

% find K using LQR
R = 1;
Q_diag = [1 1 1 1];
Q = diag(Q_diag);
K_lqr = dlqr(Ad, Bd, Q, R);

%% simulation time parameters
T_total = 20;
tspan = [0:Ts:Ts];
num_delay_timesteps = 0;

%% initialize simulation vectors
X_out = zeros(4, T_total/Ts);
X_out(:,1) = [0; 0; 0.01; 0];
u_out = zeros(1,T_total/Ts + num_delay_timesteps);
u_out(1,1) = 0;
Y_out = zeros(2,T_total/Ts);
Y_out(:,1) = Cd*X_out(:,1);
X_hat = zeros(4,T_total/Ts);
X_hat(1,1) = Y_out(1,1);
X_hat(3,1) = Y_out(2,1);

% ode solver options
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-7);

% reference signal
ref = [0; 0; 0; 0];

% weight for complementary filter
alpha = 1;

%% Simulation

for k = 2:(T_total/Ts)
    
    [t, X] = ode45(@(t, X) inv_pend_eqn_motion(t, X, u_out(1,k-1)), tspan, X_out(:,k-1));
     
    [rows, cols] = size(X);
    X_out(:,k) = X(rows,:)';
    
    Y_out(:,k) = Cd*X_out(:,k); %+ randn*[0.005; 0.0005];
    
    X_hat(1,k) = alpha*Y_out(1,k) + (1-alpha)*X_hat(1,k-1);
    X_hat(2,k) = (X_hat(1,k) - X_hat(1,k-1))/Ts;
    X_hat(3,k) = alpha*Y_out(2,k) + (1-alpha)*X_hat(3,k-1);
    X_hat(4,k) = (X_hat(3,k) - X_hat(3,k-1))/Ts;
    
    %u_out(1,k) = 0;
    u_out(1,k+num_delay_timesteps) = -K_lqr*(X_hat(:,k) - ref);
end

%% Plotting

% tiledlayout(3,2)
% nexttile
% plot(Ts:Ts:T_total, Y_out(1, :), Ts:Ts:T_total, X_hat(1, :))
% title('position')
% 
% % nexttile
% % plot(Ts:Ts:T_total, Y_out(2, :), Ts:Ts:T_total, X_hat(2, :))
% % title('velocity')
% % 
% % nexttile
% % plot(Ts:Ts:T_total, Y_out(3, :), Ts:Ts:T_total, X_hat(3, :))
% % title('angle')
% 
% nexttile
% plot(Ts:Ts:T_total, Y_out(2, :), Ts:Ts:T_total, X_hat(3, :))
% title('angular velocity')
% 
% nexttile
% plot(Ts:Ts:T_total, u_out(1, :))
% title('input force')

tiledlayout(3,2)
nexttile
plot(Ts:Ts:T_total, X_out(1, :), Ts:Ts:T_total, X_hat(1, :))
legend('Actual', 'Estimated')
title('position')

nexttile
plot(Ts:Ts:T_total, X_out(2, :), Ts:Ts:T_total, X_hat(2, :))
legend('Actual', 'Estimated')
title('velocity')

nexttile
plot(Ts:Ts:T_total, X_out(3, :))
title('angle')

nexttile
plot(Ts:Ts:T_total, X_out(4, :))
title('angular velocity')

nexttile
plot(Ts:Ts:T_total, u_out(1, 1:T_total/Ts))
title('input force')