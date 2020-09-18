function [soln] = trapCol(prb)
%TRAPCOL Performs trapezoidal collocation 
%   Detailed explanation goes here
addpath('helpers')

prb.m = size(prb.guess.state,1); %number of states
prb.p = size(prb.guess.control); %number of controls
prb = checkPrb(prb);
prb = setDefaults(prb);

fprintf('Performing Trapezoidal collocation with %d collocation points',prb.nPts);



soln = dirCol(prb);


end

