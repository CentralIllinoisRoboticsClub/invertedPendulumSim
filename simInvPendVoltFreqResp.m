%% System Model Parameters (consider moving inside InvPendVoltageInputODE.m)
% physical properties
g = 9.8; % gravity
m = 0.08; % kg, pendulum mass, meas 0.08
%Mc = 0.32; % kg, cart mass, meas 0.32, replaced with cart_vel_gain and tau
r = 0.17; % meters, center of mass location on pendulum, meas 0.17
Ip = 3.5e-3; %1/3*m1*r^2 + m2*L^2;, meas 3.5e-3
%b = 0.01; % N/(m/s), cart damping, replaced with cart_vel_gain and tau
c = 5e-4; % Nm/(rad/s), pendulum damping

% cart model parameters tau*xdd + xd = K*cmd --> X(s)/U(s) = K / ( s*(tau*s + 1) )
cart_vel_gain = 2.6*1e-3; % steady state cart velocity/cmd (meters/sec/cmd)
tau_norm = 0.07; % sec, time constant for velocity step response to cmd
tau_brake = 0.01; % used when u = cmd = 0, what about switching directions?

%% System variables
% x = cart x position, meters, positive right
% xd = x_dot
% th = pendulum angle, rad (vertical, up = 0 rad), positive CCW
% thd = th_dot
% z = [x; xd; th; thd] is the system state vector

%% Initial Conditions
x_init = 0;
xd_init = 0;
th_init = 180*pi/180;
thd_init = 0;

%% Plot settings
animate = false; % animates the pendulum system in an x-y plot
plot_line_width = 2;
tick_font = 18;
label_font = 18;

%% ************ Timing *************
Ts = 0.01; %controller feedback loop, step time for input of freq responsne
totalTime = 1.5; % Total sim time (stopped early if control and |th| > thresh
dtSim = 0.001; % sim time step (<= Ts/2)

%% Testing freq responses
tcmds = (0:Ts:totalTime);
w = 2*(2*pi);
A = 128;
cmds = A*cos(w*tcmds); % cmd (-255 to 255)
Mratio = cart_vel_gain / sqrt(w^2 + tau_norm^2*w^4);
phase = atan(1/(tau_norm*w));

ind_cmd = 1;
n_cmds = length(cmds);
cur_cmd = cmds(1);
% ************************

%% Initialize state
zinit = [x_init, xd_init, th_init, thd_init];

%% Initialize plot and playback vectors
x_all = [];
xd_all = [];
th_all = [];
thd_all = [];
cmd_all = [];
t_all = [];
t_prev = 0;

tsim = (0:dtSim:Ts)'; %local time vector for ode solver
for k = 1:round(totalTime/Ts)
  tControl = k*Ts; % global time
  if(ind_cmd <= n_cmds)
    if(tControl >= tcmds(ind_cmd))
      cur_cmd = cmds(ind_cmd);
      ind_cmd = ind_cmd + 1;
    end
  end
  u = cur_cmd; % cmd for freq response test
    
  if(u == 0)
    tau = tau_brake;
  else
    tau = tau_norm;
  end

  h = @(z,t) InvPendVoltageInputODE(z,t,g,m,r,Ip,u,tau,cart_vel_gain,c);
  z = lsode(h, zinit, tsim);

  x = z(:,1);
  xd = z(:,2);
  th = z(:,3);
  thd = z(:,4);
  zinit = [x(end), xd(end), th(end), thd(end)];
  
  t_all = [t_all; t_prev + tsim];
  t_prev = t_all(end);
  x_all = [x_all; x];
  xd_all = [xd_all; xd];
  th_all = [th_all; th];
  thd_all = [thd_all; thd];
  cmd_all = [cmd_all; tsim*0+u]; %tsim, tdebug  
end

x_resp = -A*Mratio*cos(w*t_all + phase);

hold off
figure(1)
ax = [];
ax(end+1) = subplot(3,1,1);
plot(t_all,x_all*1e3, 'linewidth',plot_line_width)
hold on
plot(t_all,x_resp*1e3, 'm', 'linewidth',plot_line_width)
ylabel('x mm', 'fontsize',label_font)
set(gca, "linewidth", plot_line_width, "fontsize", tick_font)
legend('sim','AMcos(wt+phi)')

ax(end+1) = subplot(3,1,2);
plot(t_all,xd_all*1e3, 'linewidth',plot_line_width)
hold on
plot(t_all,cmd_all,'r', 'linewidth',plot_line_width)
ylabel('xd mm/sec', 'fontsize',label_font)
set(gca, "linewidth", plot_line_width, "fontsize", tick_font)
legend('xd mm/sec', 'cmd')

ax(end+1) = subplot(3,1,3);
plot(t_all,th_all*180/pi, 'linewidth',plot_line_width); hold on
#plot([0,30],[-220,-220])
#plot([0,30],[-140,-140])
ylabel('th deg', 'fontsize',label_font)
set(gca, "linewidth", plot_line_width, "fontsize", tick_font)
linkaxes(ax,'x')
if animate
  figure(2)
  hold off
  for k = 1:10:length(t_all)
    tt = t_all(k);
    x1 = x_all(k);
    y1 = 0;
    theta = th_all(k);
    x2 = x1 - 2*r*sin(theta);
    y2 = y1 + 2*r*cos(theta);
    
    hold off
    plot([x1,x2],[y1,y2],'r-o', 'linewidth',plot_line_width);
    hold on
    plot(x1,y1,'bo','Markersize',30, 'linewidth',plot_line_width)
    xlabel('x','fontsize',label_font)
    ylabel('y','fontsize',label_font)
    xlim([-1.0,1.0])
    ylim([-1.0,1.0])
    title(tt,'fontsize',label_font)
    set(gca, "linewidth", plot_line_width, "fontsize", tick_font)
    pause(0.05)
  end
end