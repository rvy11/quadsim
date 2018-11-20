% quadsim_sensors.m
%
% Generation of sensor measurements for quadsim
%
% Inputs:
%   Forces and Moments (used to create accelerometer measurement)
%   UAV States
%   Wind vector
%   Time
%
% Outputs:
%   Sensor Measurements
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = quadsim_sensors(uu, P)

    % Extract variables from input vector uu
    %   uu = [f_and_m(1:6); x(1:12); wind_ned(1:3); time(1)];
    k=(1:6);           f_and_m=uu(k);   % Forces and Moments, body
    k=k(end)+(1:12);   x=uu(k);         % states
    k=k(end)+(1:3);    wind_ned=uu(k);  % wind vector, ned, m/s
    k=k(end)+(1);      time=uu(k);      % Simulation time, s

    % Extract forces and moments from f_and_m
    fb_x = f_and_m(1); % Total force along body x, N
    fb_y = f_and_m(2); % Total force along body y, N
    fb_z = f_and_m(3); % Total force along body z, N
    mb_x = f_and_m(4); % Total moment about body x, N-m
    mb_y = f_and_m(5); % Total moment about body y, N-m
    mb_z = f_and_m(6); % Total moment about body z, N-m

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

            pn_gps=0;
            pe_gps=0;
            alt_gps=0;
            Vn_gps=0;
            Ve_gps=0;
            Vd_gps=0;
            p_gyro=0;
            q_gyro=0;
            r_gyro=0;
            ax_accel=0;
            ay_accel=0;
            az_accel=0;
            static_press=0;
            diff_press=0;
            psi_mag=0;

    % Compile output vector
    out = [ ...
            pn_gps; ...
            pe_gps; ...
            alt_gps;  ...
            Vn_gps; ...
            Ve_gps; ...
            Vd_gps; ...
            p_gyro; ...
            q_gyro; ...
            r_gyro; ...
            ax_accel;...
            ay_accel;...
            az_accel;...
            static_press; ...
            diff_press; ...
            psi_mag;...        
            0; % future use
            0; % future use
            0; % future use
          ]; % Length: 18

end