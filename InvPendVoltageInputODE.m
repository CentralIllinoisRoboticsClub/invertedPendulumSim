
function dz = InvPendVoltageInputODE(z,t,g,m,r,Ip,u,tau,cart_vel_gain,c)

x = z(1);
xd = z(2);
th = z(3);
thd = z(4);

% A*[xdd; thdd] = B

A = [    1,           0;
     -m*r*cos(th), Ip+m*r^2 ];

B = [1/tau * (cart_vel_gain*u - xd);
     m*g*r*sin(th) - c*thd];
  
sol = A\B;

dz(1) = z(2);
dz(2) = sol(1);
dz(3) = z(4);
dz(4) = sol(2);
dz = dz';
  