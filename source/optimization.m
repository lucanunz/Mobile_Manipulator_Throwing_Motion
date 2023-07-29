clear variables
close all
syms qw q1 q2 dqw dq1 dq2 real

%% set kinematic parameters
global h1 h2 rt1 rt4 ht hb rw db mball;
hb=0.193; ht=0.797; h1=0.32; h2=0.59;
rt1=0.062; rt4=0.155; rw=0.0985; 
db=0.54; mball=3;

%% specify global constant values
global x_goal g0;
x_goal=14.6;
g0=9.81;

%% joint configuration/velocity/torque bounds
x_max=10;
q=[qw;q1;q2];
dq=[dqw;dq1;dq2];
q_max=[x_max-db/2 9*pi/10 pi/2]; % [m, rad, rad]
q_min=[0 -pi/4 -pi/4]; % [m, rad, rad]
w_max=[10.15*rw 5 5]; % [m/s, rad/s, rad/s]
w_min=-w_max; % [m/s, rad/s, rad/s]
t_max=[6 70 70]; % [N*m, N*m, N*m]
t_min=-t_max; % [N*m, N*m, N*m]

%% find the direct and differential kinematics
f = [qw-rt1+rt4+h1*cos(q1)+h2*cos(q1+q2);
     rw+hb+ht+h1*sin(q1)+h2*sin(q1+q2)];
J=jacobian(f,q);

%consider a weighting matrix since we are dealing with different unit of
%measure [m,rad,rad]. Otherwise, the result of the minimization will be
%inconsistent (we want to consider a uniform cost, independently from which joint we move)
W=diag([1/(h1*h2) 1 1]);

%% Finding best throwing configuration via optimization
%upper/lower bounds for joint configuration/velocity
ub=[q_max w_max];
lb=[q_min w_min];

%find initialization to the optimization problem which satisfies the constraints
%arbitrarily choose p_ee0 (x_ee0,z_ee0) 
%and find dp_ee0 through the balistic eqns
x_ee0=9.35;
z_ee0=1.85;
p_ee0=[x_ee0;z_ee0];
T=0.6; %arbitrary
dp_ee0=[(x_goal-p_ee0(1))/T;
        -p_ee0(2)/T + g0*T/2];

% arbitrarily choose qw_0 and find the remaining part of robot
%configuration through inverse kinematics (actually a 2R robot)
qw0=ub(1);
p0=p_ee0-[qw0-rt1+rt4;
          rw+hb+ht];
c2=(p0(1)^2+p0(2)^2-(h1^2+h2^2))/(2*h1*h2);
if (c2<-1) || (c2>1)
    disp('Desired end-effector position out of workspace');
    return
end
%positive and negative solution for q2
s2_p=sqrt(1-c2^2);  
s2_n=-sqrt(1-c2^2);  

% positive solution of q1
s1_p=(p0(2)*(h1+h2*c2)-p0(1)*h2*s2_p);
c1_p=(p0(1)*(h1+h2*c2)+p0(2)*h2*s2_p);

% negative solution of q1
s1_n=(p0(2)*(h1+h2*c2)-p0(1)*h2*s2_n);
c1_n=(p0(1)*(h1+h2*c2)+p0(2)*h2*s2_n);

%two solutions to the inverse kinematics
q0_p=[qw0; atan2(s1_p,c1_p); atan2(s2_p,c2)];
q0_n=[qw0; atan2(s1_n,c1_n); atan2(s2_n,c2)];

%choose between the two solutions the one which minimizes
%the objective function (weighted square norm of joint velocities)
J0_p=subs(J,q,q0_p);
J0_n=subs(J,q,q0_n);

%wp stands for weighted pseudoinverse
wpJ0_p=inv(W)*J0_p'*inv(J0_p*inv(W)*J0_p');
dq0_p=wpJ0_p*dp_ee0;

wpJ0_n=inv(W)*J0_n'*inv(J0_n*inv(W)*J0_n');
dq0_n=wpJ0_n*dp_ee0;

if (dq_cost_function([q0_p; dq0_p]) <= dq_cost_function([q0_n; dq0_n]))
    q0=q0_p;
    dq0=double(dq0_p);
else
    q0=q0_n;
    dq0=double(dq0_n);
end

x0=[q0; dq0];

%% plot the trajectory done by the ball with custom initialization of p_ee and dp_ee
figure
axis([6 x_goal+0.5 -0.1 3]);
daspect([1 1 1]);
plot_robot(x0);
plot_balistic(x0);

%% The result of this optimization will be used as initial guess
A = [];
b = [];
Aeq = [];
beq = [];
fun=@dq_cost_function;
nonlcon=@balistic_eq;
options=optimoptions('fmincon','Algorithm','active-set','Display','iter-detailed');
x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

%% plot the trajectory done by the ball with optimized (kinematically only) value of p_ee and dp_ee
figure
axis([6 x_goal+0.5 -0.1 3]);
daspect([1 1 1]);
plot_robot(x);
plot_balistic(x);

%% Setting parameters for single phase optimization (throwing + stopping)
global delta n_throw;
delta=0.025;
T_t=1;
n_throw=T_t/delta;
T_s=1.25;
n_stop=T_s/delta;
n=n_throw+n_stop;

%% Initialize the decision variables
% throwing phase:
%   - for the state simple linear interpolation between init and final state
%   - for the inputs set them to 0
x_init=[9.3647;1.4062;pi/2;zeros(3,1)];
x_throw=x;
des_throw=zeros(9*n_throw+6,1);
for k=1:6
    des_throw(k:9:9*n_throw+6)=linspace(x_init(k),x_throw(k),n_throw+1);
end

% stopping phase
%   - for the state simple linear interpolation between init and final state
%   - for the inputs set them to 0
x_stop=[x_throw(1)+1; pi/2; 0; 0; 0; 0];
des_stop=zeros(9*n_stop+6,1);
for k=1:6
    des_stop(k:9:9*n_stop+6)=linspace(x_throw(k),x_stop(k),n_stop+1);
end

%total decision variables to be optimized
des=[des_throw;
     des_stop(7:end,:)];

%add to upper/lower bounds also joint torque limits and repeat
q_max(1)=inf;
ub=[q_max w_max t_max];
ub=[repmat(ub,1,n) ub(1:6)];
lb=[q_min w_min t_min];
lb=[repmat(lb,1,n) lb(1:6)];

%define the cost function to use for optimization
fun=@cost_function;

%% Optimization
A = [];
b = [];
Aeq = zeros(size(des,1));
beq = zeros(size(des,1),1);

% equality constraint: fix the initial state to x_init
Aeq(1:6,1:6)=eye(6);
beq(1:6)=x_init;
% equality constraint: zeroing final joint velocities of stopping phase
Aeq(end-2:end,end-2:end)=eye(3);

nonlcon=@dyn_constraints;
options=optimoptions('fmincon','Algorithm','sqp','Display','iter-detailed', ...
                     'MaxFunctionEvaluations',100*1.86e04, 'MaxIterations',400);

[sol,cost] = fmincon(fun,des,A,b,Aeq,beq,lb,ub,nonlcon,options);
save("data/data_oneph1.mat","sol","cost","x_init","mball","db","x_goal","delta","T_t","T_s","q_max","q_min","w_max","t_max","options");