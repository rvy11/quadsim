% load_quadsim.m
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

% Bring up simulink model
open('quadsim')

% Load quadcopter UAV parameters
P = init_quadsim_params;

% Compute the trim condition and set trim parameters in P
P.delta_t0 = sqrt(P.mass*P.gravity ...
    / (4*(P.rho*P.C_prop*P.S_prop*P.k_motor^2)));