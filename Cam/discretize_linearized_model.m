%xdd = (u - m*g*th)/(M);
%thdd = (u -(M+m)*g*th)/(-M*l);

g = 9.8; % gravity
m = 0.08; % kg, pendulum mass, meas 0.08
M = 0.32; % kg, cart mass, meas 0.32
l = 0.17; % meters, center of mass location on pendulum, meas 0.17
I = 3.5e-3; %1/3*m1*r^2 + m2*L^2;, meas 3.5e-3
b = 0.01; % N/(m/s)
Ts = 0.01; %sample time in s

Ac = [0 1 0 0;
      0 0 -m*g/M 0;
     0 0 0 1;
     0 0 (M+m)*g/(M*l) 0];
 
Bc = [0;
     1/M;
     0;
     -1/(M*l)];
 
Cc = [1 0 0 0;
     0 0 1 0];
  
Dc = [0; 
     0];

sysc = ss(Ac, Bc, Cc, Dc); 
sysd = c2d(sysc, Ts);
[Ad, Bd, Cd, Dd] = ssdata(sysd)

    