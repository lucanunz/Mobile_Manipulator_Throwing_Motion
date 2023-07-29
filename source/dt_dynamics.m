function [x_f] = dt_dynamics(f,x_init,u,delta)
% given:
%   the continuous time dynamics function
%   initial state at instant t_k
%   inputs (constant over the interval [t_k;t_k+1])
%   the time interval delta=t_k+1-t_k
% return the final state at instant t_k+1

M=10;
t=delta/M;
x=x_init;
for j=1:M
    k1=f(x,u);
    k2=f(x+t/2*k1,u);
    k3=f(x+t/2*k2,u);
    k4=f(x+t*k3,u);
    x=x+t/6*(k1+2*k2+2*k3+k4);
end
x_f=x;
end