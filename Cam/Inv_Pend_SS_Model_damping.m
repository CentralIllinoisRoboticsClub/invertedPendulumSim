%% System Model Parameters 
g = 9.8; % gravity
m = 0.08; % kg, pendulum mass, meas 0.08
M = 0.32; % kg, cart mass, meas 0.32
l = 0.17; % meters, center of mass location on pendulum, meas 0.17
I = 3.5e-3; %1/3*m1*r^2 + m2*L^2;, meas 3.5e-3
b = 0.01; % N/(m/s)
Ts = 0.01; % sample time 

%% Continuous-time Linearized State Space Matrices 
% (taken from http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
mass_denom = I*(M+m)+(M*m*l^2); % denominator that shows up several times in matrices
% various elements of matrices 
a22 = -(I + m*l^2)*b/mass_denom; 
a23 = m^2*g*l^2/mass_denom;
a42 = -m*l*b/mass_denom;
a43 = m*g*l*(M+m)/mass_denom;

b21 = (I+m*l^2)/mass_denom;
b31 = m*l/mass_denom;

% matrices
Ac = [0  1   0  0; 
     0 a22 a23 0; 
     0  0   0  1;
     0 a42 a43 0];
 
Bc = [ 0
     b21;
     b31;
      0 ];
  
Cc = [1 0 0 0;
     0 0 1 0];
 
Dc = [0;
      0]; 

%% Discretized Linear State Space Matrices
% make matrices into a system and discretize them
sysc = ss(Ac, Bc, Cc, Dc); 
sysd = c2d(sysc, Ts);
% extract discrete matrices
[Ad, Bd, Cd, Dd] = ssdata(sysd);

