# invertedPendulumSim
Simulation of an inverted pendulum controller

## Simulated Controller test
Create si_gains.m with nonzero gains. 
Please do not share gains for now.   
`Kp_pend = 0; % cmd_ticks / rad`  
`Kd_pend = 0; % cmd_ticks / (rad/s)`  
`Ki_pend = 0; % cmd_ticks / (rad*s)`  

`Kp_cart = 0; % cmd_ticks / m`  
`Kd_cart = 0; % cmd_ticks / (m/s)`  
`Ki_cart = 0; % cmd_ticks / (m*s)`  

Run simInvPendVoltControl.m  
control input u = cmd

## System Model Equations
Assume the motor can control x independent of theta.  
`xdd + 1/tau * xd = 1/tau * vel_gain * cmd`  
`-m*r*cos(th) * xdd + (Ip+mr^2) * thdd = m*g*r*sin(th) - c*thd`  

Note, th = 0 when pendulum is upright

Linearize the pendulum equation for state space and transfer functions  
`-m*r * xdd + (Ip + mr^2) * thdd = m*g*r*th - c*thd`  
`xdd - (Ip + mr^2)/(m*r) * thdd = g*th - c/(m*r)*thd`  

Pending  
Add disturbance torque to the pendulum equation above

## Transfer functions
`X(s)/U(s) = K / (s*(tau*s + 1) )`  
`K = vel_gain`  

`1/tau*(K*U-sX) - (Ip+mr^2)/(mr) * s^2*TH = g*TH - c/(mr)*s*TH`  

`num_th = s*K`  
`den_th = (tau*s+1)*[(Ip+mr^2)/(mr)*s^2 + c/(mr)*s - g]`  
`TH(s)/U(s) = num_th / den_th`  

## Linearized State Space
(Using Matlab syntax to show column vectors)  
`[xd; xdd; thd; thdd] = A*[x; xd; th; thd] + B*u`  

`A = [0  1           0       0;`  
`     0 -1/tau       0       0;`  
`     0   0          0       1;`  
`     0 -1/(L1*tau)  g/L1  -c/(m*r*L1)]`  

`B = [0; K/tau; 0; K/(L1*tau)`  
`L1 = (Ip+mr^2)/(mr)`  
## Step response to u, solution to the cart equation of motion
This is for reference, not needed for control design.  
Possibly replace the ode solver with this.  
`x(t) = K*u*t + (tau*v0 - K*u*tau)*(1-exp(-t/tau)) + x0`  
`v(t) = K*u(1 - exp(-t/tau)) + v0*exp(-t/tau)`  

## Frequency Response
See simInvPendVoltFreqResp.m  
Another way to solve for tau and K = vel_gain  
input cmd = Acos(wt)  
`x_resp = Mratio1*Acos(wt+phase1)`  
Using H1(s) = X(s)/U(s):  
`Mratio1 = mag(H1) = K / sqrt(w^2 + tau^2*w^4) = K/(w*sqrt(1+tau^2*w^2))`  
`phase1 = atan2(0,K) - atan2(1,-tau*w)`  
`tan(-phase1) = -1/tau/w`  
`tau = -1/(w*tan(-phase2))`  

Can also use H2(s) = V(s)/U(s) = K / (tau*s + 1)  
`v_resp = Mratio2*Acos(wt+phase2)`  
`Mratio2 = mag(H2) = K / sqrt(1 + tau^2*w^2)`  
`phase2 = atan2(0,K) - atan2(1,tau*w)`  
`tan(-phase2) = 1/tau/w`  
`tau = 1/(w*tan(-phase2))`  

Measure phase offset in sec, delta_t.  
`period = 1/f =2*pi/w`  
`|phase_angle| = 2*pi*delta_t/period`  
`|phase_angle| = delta_t*w`  
