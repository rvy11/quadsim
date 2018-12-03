% quadsim_control.m
%
% Flight control logic for quadsim
%
% Inputs:
%   Trajectory commands
%   State Feedbacks
%   Time
%
% Outputs:
%   Control surface commands
%   Autopilot state commands (for logging and plotting)
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = quadsim_control(uu,P)

    % Extract variables from input vector uu
    %   uu = [traj_cmds(1:4); estimates(1:23); time(1)];
    k=(1:4);         traj_cmds=uu(k); % Trajectory Commands
    k=k(end)+(1:23); estimates=uu(k); % Feedback state estimates
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

    % Extract variables from traj_cmds
    h_c      = traj_cmds(1);  % commanded altitude (m)
    Vhorz_c  = traj_cmds(2);  % commanded horizontal speed (m/s) (change from uavsim)
    chi_c    = traj_cmds(3);  % commanded course (rad)
    psi_c    = traj_cmds(4);  % yaw course (rad) (change from uavsim)

    % Extract variables from estimates
    pn_hat       = estimates(1);  % inertial North position, m
    pe_hat       = estimates(2);  % inertial East position, m
    h_hat        = estimates(3);  % altitude, m
    Va_hat       = estimates(4);  % airspeed, m/s
    phi_hat      = estimates(5);  % roll angle, rad
    theta_hat    = estimates(6);  % pitch angle, rad
    psi_hat      = estimates(7);  % yaw angle, rad
    p_hat        = estimates(8);  % body frame roll rate, rad/s
    q_hat        = estimates(9);  % body frame pitch rate, rad/s
    r_hat        = estimates(10); % body frame yaw rate, rad/s
    Vn_hat       = estimates(11); % north speed, m/s
    Ve_hat       = estimates(12); % east speed, m/s
    Vd_hat       = estimates(13); % downward speed, m/s
    wn_hat       = estimates(14); % wind North, m/s
    we_hat       = estimates(15); % wind East, m/s    
    future_use   = estimates(16:23);

    % Initialize controls to trim (to be with PID logic)
    delta_e=P.delta_e0;
    delta_a=P.delta_a0;
    delta_r=P.delta_r0;
    delta_t=P.delta_t0;
    
    % Initialize autopilot commands (may be overwritten with PID logic)
    phi_c = 0;
    theta_c = 0;

    % Set "first-time" flag, which is used to initialize PID integrators
    firstTime=(time==0);

    % Flight control logic
    %   <code goes here>
    % e.g.
    %    delta_a = PID_roll_hold(phi_c, phi_hat, p_hat, firstTime, P);
    %
    % Note: For logging purposes, use variables: 
    %         Vhorz_c,  chi_c, h_c, phi_c, theta_c, psi_c

    
    % Compile vector of control surface deflections
    delta = [ ...
            delta_e; ...
            delta_a; ...
            delta_r; ...
            delta_t; ...
        ];

    % Override control delta with manual flight delta
    if P.manual_flight_flag
        error('Manual flight not supported in quadsim')
    end

    % Compile autopilot commands for logging/vis
    ap_command = [ ...
            Vhorz_c; ...
            h_c; ...
            chi_c; ...
            phi_c; ...
            theta_c; 
            psi_c; ... % change from uavsim
            0; ... % future use
            0; ... % future use
            0; ... % future use
        ];

    % Compile output vector
    out=[delta;ap_command]; % 4+9=13

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_roll_hold(phi_c, phi_hat, p_hat, init_flag, P)

    % Set up PI with rate feedback
    y_c = phi_c; % Command
    y = phi_hat; % Feedback
    y_dot = p_hat; % Rate feedback
    kp = P.roll_kp;
    ki = P.roll_ki;
    kd = P.roll_kd;
    u_lower_limit = -P.delta_a_max;
    u_upper_limit = +P.delta_a_max;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_pitch_hold(theta_c, theta_hat, q_hat, init_flag, P)

    % Set up PI with rate feedback
    y_c = theta_c; % Command
    y = theta_hat; % Feedback
    y_dot = q_hat; % Rate feedback
    kp = P.pitch_kp;
    ki = P.pitch_ki;
    kd = P.pitch_kd;
    u_lower_limit = -P.delta_e_max;
    u_upper_limit = +P.delta_e_max;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
    
end