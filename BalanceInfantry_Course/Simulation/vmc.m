
phi1=1.0944;
phi4=1.9219;
L1= 0.16;
L4=0.16;
L2=0.26;
L3=0.26;
Motor_distance = 0.11;

XD = Motor_distance + L4*cos(phi4);
XB = L1 * sin(phi1);
YD = L4 * sin(phi4);
YB = L1 * sin(phi1);
fprintf('%f',XD);
BD = sqrt((XD - XB)^2 + (YD - YB)^2);
A0 = 2*L2*(XD - XB);
B0 = 2*L2*(YD - YB);
C0 = L2^2 + BD^2 - L3^2;
LBD = sqrt((XD - XB)^2 + (YD - YB)^2);

phi_2 = 2*atan2(B0+sqrt(A0^2+B0^2-C0^2),A0+C0);

% XC = XB + L2*cos(phi_2);
% YC = 
