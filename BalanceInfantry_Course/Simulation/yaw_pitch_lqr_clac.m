
clear;

syms phi phi_dot delta_T phi_refer T pitch pitch_dot

X=[pitch 
pitch_dot];
u = [T];  %云台pitch轴,pitch角的控制,还需在最后计算总力矩时加入重力前馈1/2
A = [0 1;0 0];
m_pitch = 1.730642;%云台pitch部分的质量Kg
l = 0.1854;%与pitch轴电机连接的连杆的长度
J = (1/3)*m_pitch*l^2;%均质单摆
B = [0
    1/J];
Q_gyro = diag([5 1]);
R_gyro = [1];
K_gyro = lqr(A,B,Q_gyro,R_gyro);

Q_auto = diag([5 1]);
R_auto = [1];
K_auto = lqr(A,B,Q_auto,R_auto);
fprintf('static float  K_pitch_gyro[2] = {%f, %f} ;\n', K_gyro);
fprintf('static float  K_pitch_auto[2] = {%f, %f} ;\n', K_auto);

X=[phi 
phi_dot];
u = [T];  %云台yaw轴,yaw角的控制,还需在最后计算总力矩
A = [0 1;0 0];
R = 0.25/2;%yaw电机半径(将云台近似为圆柱)单位m,250mm
m_yaw = 3.376962;%云台整个部分的质量Kg
J = (1/2)*m_yaw*R^2;
B = [0
    1/J];
Q_gyro = diag([100 1]);
R_gyro = [12.0];
K_gyro = lqr(A,B,Q_gyro,R_gyro);

Q_auto = diag([20 10]);
R_auto = [5.0];
K_auto = lqr(A,B,Q_auto,R_auto);
fprintf('static float K_yaw_gyro[2] = {%f, %f} ;\n', K_gyro);
fprintf('static float K_yaw_auto[2] = {%f, %f} ;\n', K_auto);

