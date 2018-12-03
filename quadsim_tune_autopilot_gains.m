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
G_dr2yaw = (8*P.k_Tp*P.k_omega*P.delta_t0)/(P.Jz*s^2);

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

Gcl_pitch_low =PI_rateFeedback_TF(G_de2pitch, 5, 0.05, 5); 
Gcl_pitch_high=PI_rateFeedback_TF(H(ktheta,kde), 5, 0.05, 5); 
step(Gcl_pitch_low, Gcl_pitch_high, 10) % 2 seconds
grid on;
lgd = legend('G_de2pitch', 'H(ktheta,kde)');
lgd.FontSize = 12;
set(lgd,'string',{'G_de2pitch','H(ktheta,kde)'});
title('Step Response', 'FontSize', 14);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);