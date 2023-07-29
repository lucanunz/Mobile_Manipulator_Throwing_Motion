function [c, ceq] = balistic_eq(x)
    % given the decision variables x as inputs
    % return the non linear constraint associated to the balistic equation
    % used for the 
    global h1 h2 rt1 rt4 ht hb rw;
    global x_goal g0;

    p_ee = [x(1)-rt1+rt4+h1*cos(x(2))+h2*cos(x(2)+x(3));
            rw+hb+ht+h1*sin(x(2))+h2*sin(x(2)+x(3))];

    J = [1      -h2*sin(x(2)+x(3))-h1*sin(x(2))     -h2*sin(x(2)+x(3));
         0      h2*cos(x(2)+x(3))+h1*cos(x(2))      h2*cos(x(2)+x(3))];

    dp_ee = J*[x(4) x(5) x(6)]';

    c = [];
    ceq = [double(dp_ee(1)*dp_ee(2)/g0 + dp_ee(1)*sqrt((dp_ee(2)/g0)^2 + 2*p_ee(2)/g0) + p_ee(1) - x_goal)];
end