function f = cost_function(x)
% return cost function based on the reduction of control effort and
% impovement of robot balance
global h1 h2;
f=0;
R=eye(3);
Q=5;
P=diag([1/(h1*h2) 1 1]);
for k=1:9:size(x,1)-6
    qw_i=x(k);
    v_i=x(k+3:k+5);
    u=x(k+6:k+8);
    f=f+u'*R*u+v_i'*P*v_i+qw_i'*Q*qw_i;
end
v_i=x(end-2:end);
f=f+v_i'*P*v_i;
end