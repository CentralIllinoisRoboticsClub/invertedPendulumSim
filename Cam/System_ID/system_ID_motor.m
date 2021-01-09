clear all, clc

%% load in motor encoder data
pwm40_pulse = load('40pwm_5sec_Motor_Enc.csv');
pwm50_pulse = load('50pwm_5sec_Motor_Enc.csv');
pwm100_pulse = load('100pwm_2sec_Motor_Enc.csv');

%% manually find sample time
Ts_40 = 5/11102;
Ts_50 = 5/9848;
Ts_100 = 2/4216;
Ts = mean([Ts_40 Ts_50 Ts_100]);

t40 = 0:Ts_40:(Ts_40*(length(pwm40_pulse)-1));
t50 = 0:Ts_50:(Ts_50*(length(pwm50_pulse)-1));
t100 = 0:Ts_100:(Ts_100*(length(pwm100_pulse)-1));

%% meters per encoder increment
mpp_enc = 0.0397/320;

%% encoder increments to meters
pwm40_m = pwm40_pulse*mpp_enc;
pwm50_m = pwm50_pulse*mpp_enc;
pwm100_m = pwm100_pulse*mpp_enc;

%% estimate velocity with fixed-time interval method
pwm40_v = zeros(length(pwm40_m),1);
pwm50_v = zeros(length(pwm50_m)-1,1);
pwm100_v = zeros(length(pwm100_m)-1,1);

delta_T = 100; % half of the velocity estimation window

for i = 1:1:(length(pwm40_m))
    if i <= delta_T
        pwm40_v(i) = (pwm40_m(i+delta_T) - pwm40_m(1))/((i+delta_T)*Ts_40);
    elseif i >= (length(pwm40_m) - delta_T)
        pwm40_v(i) = (pwm40_m(length(pwm40_m)) - pwm40_m(i-delta_T))/((length(pwm40_m)-i+delta_T+1)*Ts_40);
    else
        pwm40_v(i) = (pwm40_m(i+delta_T) - pwm40_m(i-delta_T))/((delta_T*2+1)*Ts_40);
    end
end

for i = 1:1:(length(pwm50_m))
    if i <= delta_T
        pwm50_v(i) = (pwm50_m(i+delta_T) - pwm50_m(1))/((i+delta_T)*Ts_50);
    elseif i >= (length(pwm50_m) - delta_T)
        pwm50_v(i) = (pwm50_m(length(pwm50_m)) - pwm50_m(i-delta_T))/((length(pwm50_m)-i+delta_T+1)*Ts_50);
    else
        pwm50_v(i) = (pwm50_m(i+delta_T) - pwm50_m(i-delta_T))/((delta_T*2+1)*Ts_50);
    end
end

for i = 1:1:(length(pwm100_m))
    if i <= delta_T
        pwm100_v(i) = (pwm100_m(i+delta_T) - pwm100_m(1))/((i+delta_T)*Ts_100);
    elseif i >= (length(pwm100_m) - delta_T)
        pwm100_v(i) = (pwm100_m(length(pwm100_m)) - pwm100_m(i-delta_T))/((length(pwm100_m)-i+delta_T+1)*Ts_100);
    else
        pwm100_v(i) = (pwm100_m(i+delta_T) - pwm100_m(i-delta_T))/((delta_T*2+1)*Ts_100);
    end
end

%% plot positions and velocities
tiledlayout(2,3)
nexttile
plot(t40, pwm40_v)
nexttile
plot(t50, pwm50_v)
nexttile
plot(t100, pwm100_v)
nexttile
plot(t40, pwm40_m)
nexttile
plot(t50, pwm50_m)
nexttile
plot(t100, pwm100_m)

%% calculate K and tau from visual inspection
KV40 = 0.05893;
V40 = 12*(40/255);
K40 = KV40/V40;
KV40_95 = KV40*0.95;
tau40 = 0.852/3;

KV50 = 0.1362;
V50 = 12*(50/255);
K50 = KV50/V50;
KV50_95 = KV50*0.95;
tau50 = 0.5103/3;

KV100 = 0.4307;
V100 = 12*(100/255);
K100 = KV100/V100;
KV100_95 = KV100*0.95;
tau100 = 0.7381/3;

KV40 = (0.2753-0.03015)/(4.999-0.9557);
V40 = 12*(40/255);
K40_2 = KV40/V40;
KV40_95 = KV40*0.95;
tau40_2 = 0.5733/3;

KV50 = (0.2445-(-0.2908))/(4.999-0.3087);
V50 = 12*(50/255);
K50_2 = KV50/V50;
KV50_95 = KV50*0.95;
tau50_2 = 0.1158/3;

KV100 = (0.7302-0.08895)/(2-0.462);
V100 = 12*(100/255);
K100_2 = KV100/V100;
KV100_95 = KV100*0.95;
tau100_2 = 0.1779/3;
    