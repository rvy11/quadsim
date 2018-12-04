% JHU 525.661 UAV Systems & Control
% Final Quadsim Project 
% Autopilot Gains Tuning

% Creating Analytical Models
s=tf('s');
k_Fp = (P.rho*P.C_prop*P.S_prop*P.k_motor^2)/(P.k_omega^2);

G_dt2h = ((8/P.mass)*k_Fp*P.k_omega^2*P.delta_t0)/(s^2);

dy = P.L_to_motor*cos(pi/4);
dx = P.L_to_motor*sin(pi/4); 

G_de2pitch = (8*dx*k_Fp*P.k_omega^2*P.delta_t0)/(P.Jy*s^2);
G_da2roll = (8*dy*k_Fp*P.k_omega^2*P.delta_t0)/(P.Jx*s^2);
G_dr2yaw = (8*P.k_Tp*P.k_omega^2*P.delta_t0)/(P.Jz*s^2);
G_pitch2Vhx = -P.gravity/(s+(P.mu_rotorDrag/P.mass));
G_roll2Vhx = +P.gravity/(s+(P.mu_rotorDrag/P.mass));

% Testing Against Linear Models
[A B] = linearize_quadsim(P);
H = (s*eye(12)-A)\B;
H = minreal(H);
H = zpk(H);
kde=1; ktheta = 8; kpd=3; kdt=4; 
kphi=7; kda=2; kpsi=9; kdr=3;
-H(kpd,kdt);
H(ktheta,kde);
H(kphi,kda);
H(kpsi,kdr);

% subplot(1, 2, 1);
% step(G_de2pitch);
% title('Linear Step Response of G_de2pitch', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);
% subplot(1, 2, 2);
% step(H(ktheta,kde));
% title('Linear Step Response of H(ktheta,kde)', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);

% Gcl_pitch_low =PI_rateFeedback_TF(G_de2pitch, 5, 0.05, 5); 
% Gcl_pitch_high=PI_rateFeedback_TF(H(ktheta,kde), 5, 0.05, 5); 
% step(Gcl_pitch_low, Gcl_pitch_high, 10) % 2 seconds
% grid on;
% lgd = legend('G_de2pitch', 'H(ktheta,kde)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_de2pitch','H(ktheta,kde)'});
% title('Step Response', 'FontSize', 14);
% xlabel('Time', 'FontSize', 14);
% ylabel('Amplitude', 'FontSize', 14);

% Gcl_roll_low =PI_rateFeedback_TF(G_da2roll, 0.1146, 0.0075, 0.025); 
% Gcl_roll_high=PI_rateFeedback_TF(H(kphi,kda), 0.1146, 0.075, 0.025); 
% step(Gcl_roll_low, Gcl_roll_high, 10) % 2 seconds
% grid on;
% lgd = legend('G_da2roll', 'H(kphi,kda)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_da2roll','H(kphi,kda)'});
% title('Step Response', 'FontSize', 14);
% xlabel('Time', 'FontSize', 14);
% ylabel('Amplitude', 'FontSize', 14);

% Gcl_alt_low =PI_rateFeedback_TF(G_dt2h, 0.05, 0.0001, 0.05); 
% Gcl_alt_high=PI_rateFeedback_TF(-H(kpd,kdt), 0.05, 0.0001, 0.05); 
% step(Gcl_alt_low, Gcl_alt_high, 10) % 2 seconds
% grid on;
% lgd = legend('G_dt2h', '-H(kpd,kdt))');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_dt2h','-H(kpd,kdt)'});
% title('Step Response', 'FontSize', 14);
% xlabel('Time', 'FontSize', 14);
% ylabel('Amplitude', 'FontSize', 14);

% Gcl_yaw_low =PI_rateFeedback_TF(G_dr2yaw, 0.1146, 0.001, 0.075); 
% Gcl_yaw_high=PI_rateFeedback_TF(H(kpsi,kdr), 0.1146, 0.001, 0.075); 
% step(Gcl_yaw_low, Gcl_yaw_high, 10) % 2 seconds
% grid on;
% lgd = legend('G_dr2yaw', 'H(kpsi,kdr)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_dr2yaw','H(kpsi,kdr)'});
% title('Step Response', 'FontSize', 14);
% xlabel('Time', 'FontSize', 14);
% ylabel('Amplitude', 'FontSize', 14);

Gcl_pitch2Vhx =PI_rateFeedback_TF(G_pitch2Vhx, -0.0349, -0.00001, -0.0001);
Gcl_roll2Vhx =PI_rateFeedback_TF(G_roll2Vhx, 0.0349, 0.00001, 0.0001);
% step(Gcl_pitch2Vhx, G_roll2Vhx, 10) % 2 seconds
step(Gcl_pitch2Vhx, 10); % 2 seconds
grid on;
% lgd = legend('Gcl_pitch2Vhx', 'G_roll2Vhx');
lgd = legend('Gcl_pitch2Vhx');
lgd.FontSize = 12;
% set(lgd,'string',{'Gcl_pitch2Vhx','G_roll2Vhx'});
set(lgd,'string',{'Gcl_pitch2Vhx'});
title('Step Response', 'FontSize', 14);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);