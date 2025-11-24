function [T1 T2] = ykb_caLc(F0,Tp,q1,q2)

% 赋值给定变量
L1a = 0.075;
L1u = 0.15;
L1d = 0.25;
L2a = 0.075;
L2u = 0.15;
L2d = 0.25;

%中间变量计算
X1 = L1a - L1u * cos(q1);
X2 = L2a - L2u * cos(q2);
Y1 = L1u * sin(q1);
Y2 = L2u * sin(q2);
sigma1 = sqrt(-(-L1d^2-2*L1d*L2d-L2d^2+X1^2+2*X1*X2+X2^2+Y1^2-2*Y1*Y2+Y2^2) * (-L1d^2+2*L1d*L2d-L2d^2+X1^2+2*X1*X2+X2^2+Y1^2-2*Y1*Y2+Y2^2));
Xe = (Y1*sigma1-Y2*sigma1+L1d^2*X1+L1d^2*X2-L2d^2*X1-L2d^2*X2+X1*X2^2-X1^2*X2-X1*Y1^2-X1*Y2^2+X2*Y1^2+X2*Y2^2-X1^3+X2^3+2*X1*Y1*Y2-2*X2*Y1*Y2) / (2*(X1^2+2*X1*X2+X2^2+Y1^2-2*Y1*Y2+Y2^2));
Ye = (X1*sigma1+X2*sigma1-L1d^2*Y1+L1d^2*Y2+L2d^2*Y1-L2d^2*Y2+X1^2*Y1+X1^2*Y2+X2^2*Y1+X2^2*Y2-Y1*Y2^2-Y1^2*Y2+Y1^3+Y2^3+2*X1*X2*Y1+2*X1*X2*Y2) / (2*(X1^2+2*X1*X2+X2^2+Y1^2-2*Y1*Y2+Y2^2));
L = sqrt(Xe^2 + Ye^2);
sinq = Xe / L;
cosq = Ye / L;

% 雅可比矩阵元素计算
q1_L = ((Xe+X1)*sinq + (Ye-Y1)*cosq)/(L1u*(-(Xe+X1)*sin(q1) + (Ye-Y1)*cos(q1)));
q1_q = (L*((Xe+X1)*cosq - (Ye-Y1)*sinq))/(L1u*(-(Xe+X1)*sin(q1) + (Ye-Y1)*cos(q1)));
q2_L = ((Xe-X2)*sinq + (Ye-Y2)*cosq)/(L2u*((Xe-X2)*sin(q2) + (Ye-Y2)*cos(q2)));
q2_q = (L*((Xe-X2)*cosq - (Ye-Y2)*sinq))/(L2u*((Xe-X2)*sin(q2) + (Ye-Y2)*cos(q2)));

%定义矩阵A的元素(对角线)

a = q1_L;
b = q1_q;
c = q2_L;
d = q2_q;


%计算行列式

detA = a*d - b*c;

%计算逆矩阵
J_inv = (1/detA) * [d -b;-c a];

%计算J逆的转置
J = J_inv';

F = [F0;Tp];
T = J * F;
T1 = T(1,1);
T2 = T(2,1);

end