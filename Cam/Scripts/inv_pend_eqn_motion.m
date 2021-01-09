function X_dot = inv_pend_eqn_motion(t, X, u)

g = 9.8; % gravity
m = 0.08; % kg, pendulum mass, meas 0.08
M = 0.32; % kg, cart mass, meas 0.32
l = 0.17; % meters, center of mass location on pendulum, meas 0.17
b = .5; % coeff friction for cart

%u=0;
%u = -K*X;
%xdd = (u + m*l*sin(th)*thd^2 - m*g*cos(th)*sin(th))/(M + m - m*cos(th)^2);
%thdd = (u*cos(th) - (M+m)*g*sin(th) + m*l*cos(th)*sin(th)*thd^2)/(m*l*cos(th)^2 - (M+m)*l);
X_dot = zeros(4,1);

X_dot(1) = X(2);
%X_dot(2) = (u + m*l*sin(X(3))*X(4)^2 - m*g*cos(X(3))*sin(X(3)) - b*X(2))/(M + m - m*cos(X(3))^2);
X_dot(2) = (u + m*l*sin(X(3))*X(4)^2 - m*g*cos(X(3))*sin(X(3)) - b*X(2))/(M + m - m*cos(X(3))^2);     
X_dot(3) = X(4);
%X_dot(4) = (u*cos(X(3)) - (M+m)*g*sin(X(3)) + m*l*cos(X(3))*sin(X(3))*X(4)^2)/(m*l*cos(X(3))^2 - (M+m)*l);
X_dot(4) = (u*cos(X(3)) - m*g*sin(X(3))*cos(X(3))^2 + m*l*cos(X(3))*sin(X(3))*X(4)^2 - b*X(2)*cos(X(3)))/(m*l*cos(X(3))^2 - (M+m)*l) + g*sin(X(3))/l;