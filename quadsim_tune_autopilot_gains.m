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

% Creating Numerically Derived Models
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

% f1 = figure;
% step(G_de2pitch, 'r', H(ktheta,kde), 'go', 10);
% title('Linear Step Response of G_{de2pitch} and H(ktheta,kde)', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);
% grid on;
% lgd = legend('G_{de2pitch}', 'H(ktheta,kde)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_{de2pitch}','H(ktheta,kde)'});
% 
% f2 = figure;
% step(G_da2roll, 'r', H(kphi,kda), 'go', 10);
% title('Linear Step Response of G_{da2roll} and H(kphi,kda)', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);
% grid on;
% lgd = legend('G_{da2roll}', 'H(kphi,kda)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_{da2roll}','H(kphi,kda)'});
% 
% f3 = figure;
% step(G_dr2yaw, 'r', H(kpsi,kdr), 'go', 10);
% title('Linear Step Response of G_{dr2yaw} and H(kpsi,kdr)', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);
% grid on;
% lgd = legend('G_{dr2yaw}', 'H(kpsi,kdr)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_{dr2yaw}','H(kpsi,kdr)'});
% 
% f4 = figure;
% step(G_dt2h, 'r', -H(kpd,kdt), 'go', 10);
% title('Linear Step Response of G_{dt2h} and -H(kpd,kdt)', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);
% grid on;
% lgd = legend('G_{dt2h}', '-H(kpd,kdt)');
% lgd.FontSize = 12;
% set(lgd,'string',{'G_{dt2h}','-H(kpd,kdt)'});



% subplot(1, 2, 2);
% step(H(ktheta,kde), 10);
% title('Linear Step Response of H(ktheta,kde)', 'FontSize', 16);
% xlabel('Time (seconds)', 'FontSize', 14);
% ylabel('Angle Rate (degrees/second)', 'FontSize', 14);

f1 = figure;
GthetaCmd2theta=PI_rateFeedback_TF(H(ktheta,kde), 0.1146, 0.0075, 0.025); 
step(GthetaCmd2theta, 10) % 2 seconds
grid on;
title('Step Response of GthetaCmd2theta', 'FontSize', 16);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);

f2 = figure;
GphiCmd2phi=PI_rateFeedback_TF(H(kphi,kda), 0.1146, 0.075, 0.025); 
step(GphiCmd2phi, 10) % 2 seconds
grid on;
title('Step Response of GphiCmd2phi', 'FontSize', 16);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);

f3 = figure;
GhCmd2h=PI_rateFeedback_TF(-H(kpd,kdt), 0.05, 0.0001, 0.075); 
step(GhCmd2h, 10) % 2 seconds
grid on;
title('Step Response of GhCmd2h', 'FontSize', 16);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);

f4 = figure;
GyawCmd2yaw=PI_rateFeedback_TF(H(kpsi,kdr), 0.1146, 0.001, 0.075); 
step(GyawCmd2yaw, 10) % 2 seconds
grid on;
title('Step Response of GyawCmd2yaw', 'FontSize', 16);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);

f5 = figure;
Gcl_VhxCmd2Vhx =PI_rateFeedback_TF(GthetaCmd2theta*G_pitch2Vhx, -0.0349, -0.02, 0);
step(Gcl_VhxCmd2Vhx, 10); % 2 seconds
grid on;
title('Step Response of GVhxCmd2Vhx', 'FontSize', 16);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);

f6 = figure;
Gcl_Vhyc2Vhy =PI_rateFeedback_TF(GphiCmd2phi*G_roll2Vhx, 0.0349, 0.02, 0);
step(Gcl_Vhyc2Vhy, 10); % 2 seconds
grid on;
title('Step Response of GVhyCmd2Vhy', 'FontSize', 16);
xlabel('Time', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);