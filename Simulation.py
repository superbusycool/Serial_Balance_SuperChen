import mujoco
import mujoco.viewer
import numpy as np
import time
from environment import *
from VMC import *
from keyboard import *
from lqr_control import *
import math
from Controller import *


#/*位置*/
l_length_Kp = 100.0
l_length_Ki = 20.00
l_length_Kd = 0.00001
l_length_InteVal = 50
l_length_MaxVal = 150


#/*位置*/
r_length_Kp= 100.0
r_length_Ki= 20.00
r_length_Kd= 0.00001
r_length_InteVal= 50
r_length_MaxVal= 150

#/*theta相关*/
theta_Kp= 15
theta_Ki= 0
theta_Kd= 0.00001
theta_InteVal= 0
theta_MaxVal= 50

#/*yaw相关,转向采用pd控制*/
yaw_Kp= 1.0
yaw_Ki= 0
yaw_Kd= 0.00001
yaw_InteVal= 0
yaw_MaxVal= 10.0

#/*yaw相关,转向采用pd控制*/
yaw_follow_Kp= 0.25
yaw_follow_Ki= 0.0
yaw_follow_Kd= 0.000001
yaw_follow_InteVal= 0.0
yaw_follow_MaxVal= 5.0

#/*roll相关*/
roll_Kp= 3.5
roll_Ki= 0
roll_Kd= 0.00001
roll_InteVal= 0.0
roll_MaxVal= 50

m_b_whole = 8.4  #机体质量
g = 9.80665

def main():
    
    TORQUE = 1  #为1时给力矩，为0是无力矩
    GBC486 = LegWheelRobot('MJCF/env.xml')
    i = 0
    t1 = 1
    t2 = 4
    t3 = 20
    Off_Ground_Flag = 0
    # create chassis command and simple filter placeholders expected by LQR
    chassis_cmd = ChassisCmd(ctrl_mode=CHASSIS_RELAX, leg_length=0.15, leg_leng_change=LENGTH_STAY, leg_level=LEG_LOW, off_ground_flag=0, vx_set=0.0)
    chassis_kf_l = 0.0
    chassis_kf_r = 0.0
    vmc_r = leg_VMC()
    vmc_l = leg_VMC()
    keyboard = KeyboardController()
    controller = LQRController()

    Theta_PID = PID(theta_Kp, theta_Ki, theta_Kd,theta_InteVal, theta_MaxVal)
    leg_length_L_PID = PID(l_length_Kp, l_length_Ki, l_length_Kd, l_length_InteVal, l_length_MaxVal)
    leg_length_R_PID = PID(r_length_Kp, r_length_Ki, r_length_Kd, r_length_InteVal, r_length_MaxVal)
    Roll_PID = PID(roll_Kp, roll_Ki, roll_Kd, roll_InteVal, roll_MaxVal)
    Yaw_PID = PID(yaw_Kp, yaw_Ki, yaw_Kd, yaw_InteVal, yaw_MaxVal)

    Theta_PID_Output = 0
    leg_length_L_PID_Output = 0 
    leg_length_R_PID_Output = 0
    Roll_PID_Output = 0
    Yaw_PID_Output = 0

    F_bl_gravity = 0.5 * m_b_whole * g   #机体质量一半

    while True:
        i = i + 1
        
        # 执行仿真步
        GBC486.step()  # 仿真的timestep是1ms，意味着每执行一次step仿真世界时间过去1ms
        #传感器数据获取
        if i % t1 == 0: 
            GBC486.sensor_read_data()
        #vmc计算、观测器计算、LQR计算
        if i % t2 == 0:
            chassis_kf_l = GBC486.d_x    #纯轮速,无卡尔曼融合滤波
            chassis_kf_r = GBC486.d_x

            #正向运动学计算，获取状态
            vmc_r.vmc_calc_pos(phi1=GBC486.joint_pos[0]+math.pi,phi4=GBC486.joint_pos[1],pitch= GBC486.euler[1],gyro=GBC486.gyro[1])
            vmc_l.vmc_calc_pos(phi1=math.pi-GBC486.joint_pos[2],phi4=-GBC486.joint_pos[3],pitch=-GBC486.euler[1],gyro=-GBC486.gyro[1])
           
            #pid calculate
            Theta_PID_Output = Theta_PID.calc(vmc_r.theta - vmc_l.theta, 0)
            leg_length_L_PID_Output = leg_length_L_PID.calc(vmc_l.L0, chassis_cmd.leg_length)
            leg_length_R_PID_Output = leg_length_R_PID.calc(vmc_r.L0, chassis_cmd.leg_length)
            Roll_PID_Output = Roll_PID.calc(GBC486.euler[0], 0)
            Yaw_PID_Output = Yaw_PID.calc(GBC486.euler[2], 0)

            # vmc_r.F0 = Roll_PID_Output + leg_length_R_PID_Output + F_bl_gravity
            # vmc_l.F0 = -Roll_PID_Output + leg_length_L_PID_Output + F_bl_gravity
            # vmc_r.Tp = controller.UR[1] + Theta_PID_Output
            # vmc_l.Tp = controller.UL[1] - Theta_PID_Output

            vmc_r.F0 = 0
            vmc_l.F0 = 0
            vmc_r.Tp = 0
            vmc_l.Tp = 0          

            vmc_l.vmc_calc_torque()
            vmc_r.vmc_calc_torque()
        

            controller.update(vmc_l.L0, vmc_r.L0, vmc_l.theta, vmc_l.d_theta, vmc_r.theta, vmc_r.d_theta, GBC486.euler[1], GBC486.gyro[1], chassis_kf_l, chassis_kf_r, chassis_cmd)
            
            w_r = 0
            w_l = 0

            # w_r = controller.UR[0]
            # w_l = controller.UL[0]


            GBC486.wheel_torque = [w_r,w_l]
            GBC486.joint_torque = [vmc_r.torque_set[0],vmc_r.torque_set[1],vmc_l.torque_set[0],vmc_l.torque_set[1]]
            GBC486.actuator_set_torque()

        #键盘控制指令输入,以及打印数据;运行频率低以降低仿真延迟
        if i % t3 == 0:
            cmd = keyboard.get_command()
            # print(GBC486.euler[1])
            # print(vmc_r.phi1,vmc_r.phi4,vmc_l.phi1,vmc_l.phi4)
            # print(GBC486.joint_pos[0],GBC486.joint_pos[1],GBC486.joint_pos[2],GBC486.joint_pos[3])
            # print(vmc_r.L0,vmc_r.phi0,vmc_l.L0,vmc_l.phi0)
            # print(GBC486.euler[0], GBC486.euler[1], GBC486.euler[2])
            # print(Theta_PID_Output, leg_length_L_PID_Output, leg_length_R_PID_Output, Roll_PID_Output)
            # print(GBC486.d_x)



if __name__ == '__main__':
    main()

