clear all, clc

%% system parameters
a = 5.7267;
g = 9.81;
K = 0.0564;
tau = 0.049;
Ts = 0.005;
c = 0.1234;

%% linearized continuous state space
Ac = [0 1 0 0;
      0 -1/tau 0 0;
      0 0 0 1;
      0 a/tau a*g -c];
  
Bc = [0 ;
      K/tau;
      0
      -a*K/tau];
  
Cc = [1 0 0 0;
      0 0 1 0];

Dc = [0;
      0];

%% discretize linearized state space
sysc = ss(Ac, Bc, Cc, Dc); 
sysd = c2d(sysc, Ts);
[Ad, Bd, Cd, Dd] = ssdata(sysd);

%% calculate gain matrices
% find K using pole placement
p = [0.9 0.8 0.7 0.95];
K_place = place(Ad, Bd, p);

% find K using LQR
R = 0.3;
Q_diag = [1000 1 1000 1]; 
Q = diag(Q_diag);
K_lqr = dlqr(Ad, Bd, Q, R);

%% simulation time parameters
T_total = 20;
tspan = [0:Ts:Ts];
num_delay_timesteps = 0;

%% initialize simulation vectors
X_out = zeros(4, T_total/Ts);
X_out(:,1) = [0; 0; 0.2; 0];
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
    
    [t, X] = ode45(@(t, X) inv_pend_eqn_motion_test(t, X, u_out(1,k-1)), tspan, X_out(:,k-1));
    [rows, cols] = size(X);
    X_out(:,k) = X(rows,:)';
    Y_out(:,k) = Cd*X_out(:,k);
    
    X_hat(1,k) = alpha*Y_out(1,k) + (1-alpha)*X_hat(1,k-1);
    X_hat(2,k) = (X_hat(1,k) - X_hat(1,k-1))/Ts;
    X_hat(3,k) = alpha*Y_out(2,k) + (1-alpha)*X_hat(3,k-1);
    X_hat(4,k) = (X_hat(3,k) - X_hat(3,k-1))/Ts;
    
    u_out(1,k) = -K_lqr*(X_hat(:,k) - ref);
    if u_out(1,k) > 12
        u_out(1,k) = 12;
    elseif u_out(1,k) < -12
        u_out(1,k) = -12;
    end
    %u_out(1,k) = 0;

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
plot(Ts:Ts:T_total, X_out(1, :))
legend('Actual')
title('position')

nexttile
plot(Ts:Ts:T_total, X_out(2, :))
legend('Actual')
title('velocity')

nexttile
plot(Ts:Ts:T_total, X_out(3, :))
title('angle')

nexttile
plot(Ts:Ts:T_total, X_out(4, :))
title('angular velocity')

nexttile
plot(Ts:Ts:T_total, u_out(1, 1:T_total/Ts))
title('input voltage')