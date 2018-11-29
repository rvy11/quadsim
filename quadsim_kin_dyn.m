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
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);
    Pdot_ned = transpose(R_ned2b)*vg_b;
    vgdot_b = cross(-w_b, vg_b)+((1/P.mass)*f_b);
    body_to_euler = [1  sin(phi)*tan(theta) cos(phi)*tan(theta); 
                     0      cos(phi)            -sin(phi); 
                     0  sin(phi)*sec(theta) cos(phi)*sec(theta)];
    euler_rates = body_to_euler*w_b;
    J = [P.Jx       0        -P.Jxz; 
            0      P.Jy         0; 
        -P.Jxz      0         P.Jz];
    wdot_b = J\(cross(-w_b,J*w_b)+m_b);

    % Compile state derivatives vector
    xdot = [Pdot_ned; vgdot_b; euler_rates; wdot_b];

    % Compile function ouput
    out = xdot;

end
