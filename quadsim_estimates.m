% quadsim_estimates.m
%
% Generation of feedback state estimates for quadsim
%
% Inputs:
%   Measurements
%   Time
%
% Outputs:
%   Feedback state estimates
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = quadsim_estimates(uu,P)

    % Extract variables from input vector uu
    %   uu = [meas(1:18); time(1)];
    k=(1:18);               meas=uu(k);   % Sensor Measurements
    k=k(end)+(1);           time=uu(k);   % Simulation time, s

    % Extract mesurements
    k=1;
    pn_gps = meas(k); k=k+1; % GPS North Measurement, m
    pe_gps = meas(k); k=k+1; % GPS East Measurement, m
    alt_gps= meas(k); k=k+1; % GPS Altitude Measurement, m
    Vn_gps = meas(k); k=k+1; % GPS North Speed Measurement, m/s
    Ve_gps = meas(k); k=k+1; % GPS East Speed Measurement, m/s
    Vd_gps = meas(k); k=k+1; % GPS Downward Speed Measurement, m/s
    p_gyro = meas(k); k=k+1; % Gyro Body Rate Meas. about x, rad/s
    q_gyro = meas(k); k=k+1; % Gyro Body Rate Meas. about y, rad/s
    r_gyro = meas(k); k=k+1; % Gyro Body Rate Meas. about z, rad/s
    ax_accel = meas(k); k=k+1; % Accelerometer Meas along x, m/s/s
    ay_accel = meas(k); k=k+1; % Accelerometer Meas along y, m/s/s
    az_accel = meas(k); k=k+1; % Accelerometer Meas along z, m/s/s
    static_press = meas(k); k=k+1; % Static Pressure Meas., N/m^2
    diff_press = meas(k); k=k+1; % Differential Pressure Meas., N/m^2
    psi_mag = meas(k); k=k+1; % Yaw Meas. from Magnetometer, rad
    future_use = meas(k); k=k+1;
    future_use = meas(k); k=k+1;
    future_use = meas(k); k=k+1;

            pn_hat=0;
            pe_hat=0;
            h_hat=0;
            Va_hat=0;
            phi_hat=0;
            theta_hat=0;
            psi_hat=0;
            p_hat=0;
            q_hat=0;
            r_hat=0;
            Vn_hat=0;
            Ve_hat=0;
            Vd_hat=0;
            wn_hat=0;
            we_hat=0;
            phi_hat_unc=0;
            theta_hat_unc=0;
    
    % Compile output vector
    out = [...
            pn_hat;...    % 1
            pe_hat;...    % 2
            h_hat;...     % 3
            Va_hat;...    % 4
            phi_hat;...   % 5
            theta_hat;... % 6
            psi_hat;...   % 7
            p_hat;...     % 8
            q_hat;...     % 9 
            r_hat;...     % 10
            Vn_hat;...    % 11
            Ve_hat;...    % 12
            Vd_hat;...    % 13
            wn_hat;...    % 14
            we_hat;...    % 15
            phi_hat_unc;...   % 16
            theta_hat_unc;... % 17
            0; % future use
            0; % future use
            0; % future use
            0; % future use
            0; % future use
            0; % future use
        ]; % Length: 23
    
end 

function y = LPF(u,yPrev,tau,Ts)
%
%  Y(s)       a           1
% ------ = ------- = -----------,  tau: Filter time contsant, s
%  U(s)     s + a     tau*s + 1         ( tau = 1/a )
%

alpha_LPF = 0;
y = 0;

end
