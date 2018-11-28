function Gcl = PI_rateFeedback_TF(Gplant,kp,ki,kd)
% PI_rateFeedback_TF - Transfer Function for PI with rate feedback

s=tf('s');

% Closed Loop Transfer Function from Input to Output
Ginner = (Gplant)/(1+(Gplant*kd*s));
Gcl = ((kp+(ki/s))*Ginner)/(1+((kp+(ki/s))*Ginner));

% Minimum Realization (numerically cancel matching poles and zeros)
Gcl      = minreal(Gcl);
