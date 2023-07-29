function [x_traj,z_traj]=plot_balistic(x)

%% kinematic parameters
global h1 h2 rt1 rt4 ht hb rw;
global x_goal g0;

p_ee=[x(1)-rt1+rt4+h1*cos(x(2))+h2*cos(x(2)+x(3));
     rw+hb+ht+h1*sin(x(2))+h2*sin(x(2)+x(3));
     0;
     1];

%% compute the ee velocity via differential kinematics
J = [1      -h2*sin(x(2)+x(3))-h1*sin(x(2))     -h2*sin(x(2)+x(3));
     0      h2*cos(x(2)+x(3))+h1*cos(x(2))      h2*cos(x(2)+x(3))];
dp_ee=J*x(4:6);

%% compute the trajectory done by the ball
T=(x_goal-p_ee(1))/dp_ee(1);
t=0:0.001:T;
x_traj=p_ee(1)+dp_ee(1).*t;
z_traj=p_ee(2)+dp_ee(2).*t-0.5*g0.*t.^2;

%compute the weighted norm of the solution
wnorm=sqrt(dq_cost_function(x));

%% plot
hold on
xline(x_goal,'Color','r','LineWidth',0.03,'LineStyle','--','Label','goal')
yline(0,'Color','k','LineStyle','-','LineWidth',0.01)
plot(x_traj,z_traj,'Color','b'),xlabel('x'),ylabel('z'),grid on
end