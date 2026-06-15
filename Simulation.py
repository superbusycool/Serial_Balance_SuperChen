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
import matplotlib.pyplot as plt


#/*位置*/
l_length_Kp = 250.0
l_length_Ki = 10.00
l_length_Kd = 0.00001
l_length_InteVal = 0
l_length_MaxVal = 200


#/*位置*/
r_length_Kp= 250.0
r_length_Ki= 0.00
r_length_Kd= 0.00001
r_length_InteVal= 0
r_length_MaxVal= 200

#/*theta相关*/
theta_Kp= 50.0
theta_Ki= 10.0
theta_Kd= 0.00001
theta_InteVal= 0
theta_MaxVal= 10.0

#/*yaw相关,转向采用pd控制*/
yaw_Kp= 3.0
yaw_Ki= 0
yaw_Kd= 0.00001
yaw_InteVal= 0
yaw_MaxVal= 10.0


#/*roll相关*/
roll_Kp= 5.0
roll_Ki= 0.1
roll_Kd= 0.00001
roll_InteVal= 10.0
roll_MaxVal= 40

m_b_whole = 8.4  #机体质量
g = 9.80665

def main():

    chassis_kf_l = 0.0
    chassis_kf_r = 0.0
    yaw_target = 0.0

    TORQUE = 1  #为1时给力矩，为0是无力矩
    GBC486 = LegWheelRobot('MJCF/env.xml')
    i = 0
    t1 = 1
    t2 = 4
    t3 = 20
    Off_Ground_Flag = 0
    # create chassis command and simple filter placeholders expected by LQR
    chassis_cmd = ChassisCmd(ctrl_mode=CHASSIS_OPENLOOP, leg_length=0.20, leg_leng_change=LENGTH_STAY, leg_level=LEG_LOW, off_ground_flag=0, vx_set=0, vw_set=0)
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

    # # 实时曲线绘图设置
    # plt.ion()
    # fig, ax = plt.subplots(figsize=(10, 4))
    # pitch_data = []
    # time_data = []
    # line, = ax.plot([], [], 'b-', linewidth=1)
    # ax.set_xlabel('Time (s)')
    # ax.set_ylabel('Pitch (rad)')
    # ax.set_title('IMU Pitch Angle')
    # ax.grid(True)
    # sim_start_time = time.time()

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
           
            if(abs(GBC486.euler[1])> 0.2):
               yaw_target = GBC486.euler[2]
            yaw_target  -= chassis_cmd.vw_set*0.05  #根据转向速度指令调整yaw目标值

            #pid calculate
            Theta_PID_Output = Theta_PID.calc(vmc_r.theta - vmc_l.theta, 0)
            leg_length_L_PID_Output = leg_length_L_PID.calc(vmc_l.L0, chassis_cmd.leg_length)
            leg_length_R_PID_Output = leg_length_R_PID.calc(vmc_r.L0, chassis_cmd.leg_length)
            Roll_PID_Output = Roll_PID.calc(GBC486.euler[0]*(180.0/math.pi), 0)
            Yaw_PID_Output = Yaw_PID.calc(GBC486.euler[2], yaw_target) 

            vmc_r.F0 = -Roll_PID_Output + leg_length_R_PID_Output + F_bl_gravity
            vmc_l.F0 = Roll_PID_Output + leg_length_L_PID_Output + F_bl_gravity
            vmc_r.Tp = controller.UR[1] - Theta_PID_Output
            vmc_l.Tp = controller.UL[1] + Theta_PID_Output

            # vmc_r.F0 = 0
            # vmc_l.F0 = 0
            # vmc_r.Tp = 0
            # vmc_l.Tp = 0          

            vmc_l.vmc_calc_torque()
            vmc_r.vmc_calc_torque()
        

            controller.update(vmc_l.L0, vmc_r.L0, vmc_l.theta, vmc_l.d_theta, vmc_r.theta, vmc_r.d_theta, GBC486.euler[1], GBC486.gyro[1], chassis_kf_l, chassis_kf_r, chassis_cmd)
            
            # w_r = 0
            # w_l = 0

            # w_r = controller.UR[0]
            # w_l = controller.UL[0]

            w_r = controller.UR[0] - Yaw_PID_Output
            w_l = controller.UL[0] + Yaw_PID_Output


            GBC486.wheel_torque = [-w_r,-w_l]
            GBC486.joint_torque = [vmc_r.torque_set[0],vmc_r.torque_set[1],-vmc_l.torque_set[0],-vmc_l.torque_set[1]]
            GBC486.actuator_set_torque()

        #键盘控制指令输入,以及打印数据;运行频率低以降低仿真延迟
        if i % t3 == 0:
            #键盘控制信息
            cmd = keyboard.get_command()
            chassis_cmd.vx_set = cmd[0] * 2.0  # 前向速度指令，范围[-1, 1]，映射到实际速度范围
            chassis_cmd.vw_set = cmd[1]* 0.2  # 转向速度指令，范围[-1, 1]，映射到实际转向速度范围
            #

            # print(GBC486.euler[1])
            # print(vmc_r.phi1,vmc_r.phi4,vmc_l.phi1,vmc_l.phi4)
            # print(GBC486.joint_pos[0],GBC486.joint_pos[1],GBC486.joint_pos[2],GBC486.joint_pos[3])
            # print(vmc_r.L0,vmc_l.L0,GBC486.euler[0],Roll_PID_Output)
            # print(GBC486.euler[0], GBC486.euler[1], GBC486.euler[2])
            # print(Theta_PID_Output, leg_length_L_PID_Output, leg_length_R_PID_Output, Roll_PID_Output)
            # print(GBC486.d_x)
            # print(vmc_r.phi0,vmc_r.theta,vmc_l.phi0,vmc_l.theta)
            # print(vmc_r.torque_set[0],vmc_r.torque_set[1],vmc_l.torque_set[0],vmc_l.torque_set[1])
            # print(controller.UR[0],controller.UL[0])
            # print(vmc_r.L0,vmc_l.L0,chassis_cmd.leg_length)
            # print(vmc_r.theta,vmc_l.theta)

            print(GBC486.euler[1])

            # 实时绘制pitch角度曲线
            # pitch_data.append(GBC486.euler[1])
            # elapsed = time.time() - sim_start_time
            # time_data.append(elapsed)
            # line.set_xdata(time_data)
            # line.set_ydata(pitch_data)
            # ax.relim()
            # ax.autoscale_view()
            # if len(time_data) > 1:
            #     ax.set_xlim(time_data[0], time_data[-1] + 0.5)
            # plt.pause(0.001)



if __name__ == '__main__':
    main()

