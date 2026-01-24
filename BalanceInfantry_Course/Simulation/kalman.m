% H = [1 2 ;3  1];
% H_inv = inv(H);
% % H_inv = pinv(H);%求伪逆
% H_inv
% H'
A=[1 0.003;0 1];
Q = diag([50,15]);
I = diag([1,1]);
R = diag([5,2]);
P_0= diag([1,1]);
H = diag([1,1]);
F = P_0*H'/(H*P_0*H'+R);
P_1 = (I-F*H)*P_0;
P_2= A*P_1*A'+ Q;
P_2
F_2 = P_2*H'/(H*P_2*H'+R);
P_3 = (I-F_2*H)*P_2;
P_4= A*P_3*A'+ Q;
P_4
F_3 = P_4*H'/(H*P_4*H'+R);
P_5 = (I-F_3*H)*P_4;
P_6= A*P_5*A'+ Q;
P_6
F_4 = P_6*H'/(H*P_6*H'+R);
P_7 = (I-F_4*H)*P_6;
P_8= A*P_7*A'+ Q;
P_8