clear all; close all; clc;
addpath ../

%Template for an optimal control problem to be solved by opt-m
%(This problem does not run)

%Create contraints
% 1)Dynamics   2)Path Constraint   3)boundary constraint

p.k = 1; 
p.c = 0.1;
prb.cnstr.dyn = @(t,x,u)( dynPendulum(x,u,p) ); % of the form [x_dot] = f(t,x,u)
prb.cnstr.path = []; %myPathFunc; of the form [c,c_ceq] = g(t,x,u)
prb.cnstr.bound = []; %myBoundFunc; of the form [c,c_eq] = h(to,xo,tf,xf)

%Set initial guess: all must have same size(,2)

prb.guess.time = [0 1];  %some guess at end time
prb.guess.state = [0 pi; pi pi]; %linear guess from initial to final state
prb.guess.control = [0 0]; %linear guess on control

%Create bounds
%initial/final and all state, initial/final time, all control
% set to -inf or inf for lower or upper respectively if there is no bound

prb.bnd.tiLow = 0;                %initial time lower bound
prb.bnd.tiUpp = 0;                %initial time upper bound
prb.bnd.tfLow = .5;              %final time lower bound
prb.bnd.tfUpp = 2.5;              %final time upper bound

prb.bnd.xiLow = [0 0]';       %initial state lower bound
prb.bnd.xiUpp = [0 0]';       %initial state upper bound
prb.bnd.xfLow = [pi 0]';       %final state lower bound
prb.bnd.xfUpp = [pi 0]';       %final state upper bound
prb.bnd.xLow = [-2*pi -inf]';    %for all state but ^^^^ lower bound
prb.bnd.xUpp = [2*pi inf]';        %for all state but ^^^^ upper bound

prb.bnd.uLow = -5;            %for all control lower bound
prb.bnd.uUpp = 5;           %for all control upper bound


%Define Objective(cost) Funcitons
%Requires one or both:  1)Path Objective   2)Boundary Objective

prb.obj.path = @(t,x,u)( u.^2); %myPathObjective; 
                                   % of the form w = w(t,x,u) where
                                   % w is some scalar cost 
                                   % integrated over the trajectory
prb.obj.boundary = []; %myBoundaryObjective; 
                                   % of the form v = k(to,xo,tf,xf)
                                   % where v is some scalar cost assocaited
                                   % with the trajectory bounds


%Set solution accuracy (
prb.acc = 'med'; %'low', 'medium', or 'high'  will default to low

%Set number of collocation points
prb.nPts = []; %will default given some accuracy


soln = trapCol(prb);

t = linspace(soln.t(1),soln.t(2),50);
x = soln.xFunc(t);
plot(t,x(1,:),'LineWidth', 2)
hold on
title('time vs state for pendulum')
xlabel('time')
ylabel('state')





