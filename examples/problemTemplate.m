clear all; close all; clc;
addpath ../

%Template for an optimal control problem to be solved by opt-m
%(This problem does not run)

%Create contraints
% 1)Dynamics   2)Path Constraint   3)boundary constraint

prb.cnstr.dyn = myDynamicsFunc; % of the form [x_dot] = f(t,x,u)
prb.cnstr.path = []; %myPathFunc; of the form [c,c_ceq] = g(t,x,u)
prb.cnstr.bbound = []; %myBoundFunc; of the form [c,c_eq] = h(to,xo,tf,xf)

%Set initial guess

prb.guess.time = [0 150];  %some guess at end time
prb.guess.state = [xo xf]; %linear guess from initial to final state
prb.guess.control = [uo uf]; %linear guess on control

%Create bounds
%initial/final and all state, initial/final time, all control
% set to -inf or inf for lower or upper respectively if there is no bound

prb.bnd.tiLow = 0;                %initial time lower bound
prb.bnd.tiUpp = 0;                %initial time upper bound
prb.bnd.tfLow = 100;              %final time lower bound
prb.bnd.tfUpp = 200;              %final time upper bound

prb.bnd.xiLow = [0 0 0 0]';       %initial state lower bound
prb.bnd.xiUpp = [0 0 0 0]';       %initial state upper bound
prb.bnd.xfLow = [1 1 1 1]';       %final state lower bound
prb.bnd.xfUpp = [3 3 3 3]';       %final state upper bound
prb.bnd.xLow = [-1 -1 -1 -1]';    %for all state but ^^^^ lower bound
prb.bnd.xUpp = [4 4 4 4]';        %for all state but ^^^^ upper bound

prb.bnd.uLow = [0 0]';            %for all control lower bound
prb.bnd.uUpp = [10 10]';           %for all control upper bound


%Define Objective(cost) Funcitons
%Requires one or both:  1)Path Objective   2)Boundary Objective

prb.obj.path = []; %myPathObjective; 
                                   % of the form w = j(t,x,u) where
                                   % w is some scalar cost 
                                   % integrated over the trajectory
prb.obj.boundary = []; %myBoundaryObjective; 
                                   % of the form v = k(to,xo,tf,xf)
                                   % where v is some scalar cost assocaited
                                   % with the trajectory bounds


%Set solution accuracy (
prb.acc = []; %'low', 'medium', or 'high'  will default to low

%Set number of collocation points
prb.nPts = []; %will default given some accuracy








