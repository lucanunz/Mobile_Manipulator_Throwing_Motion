function f = dq_cost_function(x)
global h1 h2;
%consider a weighting matrix since we are dealing with different unit
%measures [m,rad,rad] and the result of the minimization will be
%inconsistent otherwise (we want to consider a uniform cost, independently from which joint we move)
W=diag([1/(h1*h2) 1 1]);
f=(x(4:6)'*W*x(4:6));
end