function [dq] = MAVdynamics(q,u)
%MAVDYNAMICS Dynamics of a low mass Mars Ascent Vehicle
%   Detailed explanation goes here
dx = 1;
dVx = 1;
dh = 1;
dVh = 1;
dm = 1;

dq = [dx;dVx;dh;dVh;dm];
end

