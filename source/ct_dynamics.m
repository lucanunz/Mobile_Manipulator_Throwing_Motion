function [x_dot] = ct_dynamics(x,u)
% return the continuous time dynamics given the state and the input
% the input x=(xb,th1,th2,dxb,dth1,dth2)
global mball;
[iM,m,E,G]=get_dyn_terms(mball,x(2),x(3),x(5),x(6));

ni=x(4:6);
x_dot=[G*ni;
       iM*(E*u-m)];
end