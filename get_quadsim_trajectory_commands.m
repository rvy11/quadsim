function [WP_n, WP_e, h_c, psi_c] = get_quadsim_trajectory_commands(time)
% USAGE: 
%    In quadsim_control.m:
%       [WP_n, WP_e, h_c, psi_c] = get_quadsim_trajectory_commands(time);
%                           \
%                            `--- Radians

% Specify time-based commands
cmd_grid = [ ... 
    ... % time   WP_n   WP_e   h_c   psi_c_deg
       -inf      0      0      50     0       ; ...
          5      0      0      50     60      ; ...
          10     0      0      40     60      ; ...
          15     40     0      40     60      ; ...
          35     40    60      40     60      ; ...
          40     40    60      40    300      ; ...
          45     40    60     100    300      ; ...
          60    -40    80      40    300      ; ...
         100     0     0      100      0      ; ...
          ];

% Find current command index based on time      
k=find(time>=cmd_grid(:,1),1,'last');

% Set commands
WP_n = cmd_grid(k,2);
WP_e = cmd_grid(k,3);
h_c  = cmd_grid(k,4);
psi_c= cmd_grid(k,5)*pi/180;
    