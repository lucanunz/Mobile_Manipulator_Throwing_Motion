function [c, ceq] = dyn_constraints(x)
    % given the decision variables (states and inputs sequence)
    % return the equality constraint of the discretized dynamic model
    % and the constraint for the balance
global delta mball n_throw;
x_throw=x(9*n_throw+1:9*n_throw+6);
[c,ceq] = balistic_eq(x_throw);
for k=1:9:size(x,1)-6
    if (k==9*n_throw+1)
        mball=0;
    end
    x_i=x(k:k+5);
    u=x(k+6:k+8);
    x_f=dt_dynamics(@ct_dynamics,x_i,u,delta);
    x_next=x(k+9:k+14);
    ceq=[ceq; x_next-x_f];
    [D,b]=get_balance_terms(mball,x_i(2),x_i(3),x_i(5),x_i(6));
    c=[c; D*u-b];
end
mball=3;
end