%(M + m)*xdd + b*xd + m*l*thdd*cos(th) - m*l*thd^2*sin(th) = u
%(I + m*l^2)*thdd + m*g*l*sin(th) = -m*l*xdd*cos(th)
%x1 = x
%x2 = xd
%x3 = th
%x4 = thd

%x1d = xd
%x2d = xdd = (-b*xd - m*l*thdd*cos(th) + m*l*thd^2*sin(th) + u)/(M+m)
%    = (-b*x2 - m*l*x4d*cos(x3) + m*l*x4^2*sin(x3) + u)/(M+m)