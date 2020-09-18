function [soln] = trapCol(prb)
%TRAPCOL Performs trapezoidal collocation 
%   Detailed explanation goes here
addpath('helpers')

prb.m = size(prb.guess.state,1); %number of states
prb.p = size(prb.guess.control); %number of controls
prb = checkPrb(prb);
prb = setDefaults(prb);

fprintf('Performing Trapezoidal collocation with %d collocation points',prb.nPts);

%set weights for trapezoidal quadrature over path objective
%derived in README
prb.cnstr.weights = ones(prb.nPnts);
prb.cnstr.weights(1,end) = .5;


prb.cnstr.dynErr = @findDynErr;

soln = dirCol(prb);


end

function [dynErr] = findDynErr(hk,x,f)
%given the timestep, state, and dynamics, we can compute the equality
%constraint necessary to enfore the dynamics. x_dot(t) = f(t).
%derivation in README

x_kPlus1 = x(:,2:end);
x_k = x(:,1:end-1);

f_kPlus1 = f(:,2:end);
f_k = f(:,1:end-1);

%nonlinear equality constraint enforcing dynamics. added to c_eq in dirCol
dynErr = x_k - x_kPlus1 +.5*hk*(f_kPlus1 + f_k); 
end




