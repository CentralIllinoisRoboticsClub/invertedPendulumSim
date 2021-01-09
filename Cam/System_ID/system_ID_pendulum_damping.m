clear all, clc

%% manually find sample rate (duration_of_collection/num_data_pts)
trial_1 = 7.57/15254;
trial_2 = 15.32/30374;
trial_3 = 30.30/59808;

Ts = mean([trial_1 trial_2 trial_3]);
%% load and plot pendulum swinging data 
theta_5deg = load('5_deg_pend_enc.csv');
theta_20deg = load('20_deg_pend_enc.csv');

t_5deg = Ts:Ts:(length(theta_5deg)*Ts);
t_20deg = Ts:Ts:(length(theta_20deg)*Ts);

tiledlayout(2,1)
nexttile
title('5 degrees')
plot(t_5deg, theta_5deg)

nexttile
title('20 degrees')
plot(t_20deg, theta_20deg)

%% calculate parameters

% period
Td_data = [(2.226-1.341) (4.357-3.499) (1.542-0.7531) (2.73-1.932) (15.98-15.14) (40.35-39.49)];
Td = mean(Td_data);

% damped natural frequency
w_d = 2*pi/Td;

% amplitude
N = 3;
phiAmp_ratio1 = (21.15+20.4)/(19.35+18.6); %20 deg trial
phiAmp_ratio2 = (4.8+5.25)/(3.75+4.35);    %5 deg trial
phiAmp_ratio = mean([phiAmp_ratio1 phiAmp_ratio2]);

% damping term (zeta)
f = 1/(2*pi*N) * log(phiAmp_ratio);

% natural frequency
w_n = w_d/sqrt(1-f^2);

% 1% settling time
Tset = 4.6/(f*w_n);

% damping coeff
c1 = 2*f*w_n;

%effective length
g = 9.81;
L = g/w_n^2;

% A = (m*l)/(I + m*l^2)
A = 1/L;
