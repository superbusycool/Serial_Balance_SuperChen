clear;

syms phi phi_dot 

 x = [phi
phi_dot];   %此处phi为偏航角,u=[delta_T]
Rw = 0.1;  %驱动轮半径m
Rl = 0.165; %轮距的一半m
L = 0.06; %驱动轮的宽度,算上lk电机的总宽度m
mw = 1.635;%驱动轮质量1.635Kg
Iw = (1/2)*mw*Rw^2;   %轮子的转动惯量
M1 = 6.97;%机体质量无云台Kg
I = M1*(0.330^2+0.298^2)/12.0;  %机体偏航角旋转转轴的转动惯量

A1 = [0];
B1 = [1];
Q1 = diag(10);
R1 = diag(1);
K1=lqr(A1,B1,Q1,R1);
fprintf('k1= %f\n',K1);

A2 = [0 1;0 0];
b1 = (Rw*Rl) / (2*Rl^2*Iw+Rw^2*I);
B2 = [0 
    b1];%转置
Q2 = diag([1500 600]);
R2 = diag(1.75);
K2=lqr(A2,B2,Q2,R2);
fprintf('K2 = [%f, %f]\n', K2); %K 的行数 = 控制输入数量（B 的列数）K的列数 = 状态变量数量（A 的行数）