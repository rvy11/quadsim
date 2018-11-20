% quadsim_kin_dyn.m
%
% Kinematics and Dynamics for quadsim
%
% Inputs:
%   UAV States
%   Forces and Moments (used to create accelerometer measurement)
%   Time
%
% Outputs:
%   Vector of state derivatives
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = quadsim_kin_dyn(uu,P)

    % Extract variables from input vector uu
    %   uu = [x(1:12); f_and_m(1:6); time(1)];
    k=1:12;          x=uu(k);         % States
    k=k(end)+(1:6);  f_and_m=uu(k);   % Forces and Moments, body
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

    % Extract state variables from x
    pn    = x(1);   % North position, m
    pe    = x(2);   % East position, m
    pd    = x(3);   % Down position, m
    u     = x(4);   % body-x groundspeed component, m/s
    v     = x(5);   % body-y groundspeed component, m/s
    w     = x(6);   % body-z groundspeed component, m/s
    phi   = x(7);   % EulerAngle: roll, rad
    theta = x(8);   % EulerAngle: pitch, rad
    psi   = x(9);   % EulerAngle: yaw, rad
    p     = x(10);  % body rate about x, rad/s
    q     = x(11);  % body rate about y, rad/s
    r     = x(12);  % body rate about z, rad/s

    % Combine states to vector form for convenience
    P_ned = [pn; pe; pd];   % NED position, m
    vg_b  = [u; v; w];      % Groundspeed vector, body frame, m/s
    w_b   = [p; q; r];      % body rates about x,y,z, rad/s

    % Extract body-frame forces and moments
    f_b   = f_and_m(1:3); % External forces along body x,y,z, N
    m_b   = f_and_m(4:6); % External moments about body x,y,z, N-m

    % Your code goes below...
        
    Pdot_ned = zeros(3,1);
    vgdot_b = zeros(3,1);
    euler_rates = zeros(3,1);    
    wdot_b = zeros(3,1);

    % Compile state derivatives vector
    xdot = [Pdot_ned; vgdot_b; euler_rates; wdot_b];

    % Compile function ouput
    out = xdot;

end
