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
def main():
    
    TORQUE = 1  #为1时给力矩，为0是无力矩
    GBC486 = LegWheelRobot('MJCF/env.xml')
    i = 0
    t1 = 1
    t2 = 4
    t3 = 20
    Off_Ground_Flag = 0
    # create chassis command and simple filter placeholders expected by LQR
    chassis_cmd = ChassisCmd(ctrl_mode=CHASSIS_RELAX, leg_level=LEG_MID, leg_leng_change=LENGTH_STAY, vx_set=0.0)
    chassis_kf_l = ChassisKF(0.0)
    chassis_kf_r = ChassisKF(0.0)
    vmc_r = leg_VMC()
    vmc_l = leg_VMC()
    keyboard = KeyboardController()
    controller = LQRController()


    while True:
        i = i + 1
        
        # 执行仿真步
        GBC486.step()  # 仿真的timestep是1ms，意味着每执行一次step仿真世界时间过去1ms
        #传感器数据获取
        if i % t1 == 0: 
            GBC486.sensor_read_data()
        #vmc计算、观测器计算、LQR计算
        if i % t2 == 0:
            #正向运动学计算，获取状态
            vmc_r.vmc_calc_pos(phi1=GBC486.joint_pos[0]+math.pi,phi4=GBC486.joint_pos[1],pitch= GBC486.euler[1],gyro=GBC486.gyro[1])
            vmc_l.vmc_calc_pos(phi1=math.pi-GBC486.joint_pos[2],phi4=-GBC486.joint_pos[3],pitch=-GBC486.euler[1],gyro=-GBC486.gyro[1])
            vmc_r.F0 = 0
            vmc_l.F0 = 0
            vmc_r.Tp = 0
            vmc_l.Tp = 0          

            vmc_l.vmc_calc_torque()
            vmc_r.vmc_calc_torque()
            # vmc.vmc_calc()

            controller.update(vmc_l.L0, vmc_r.L0, vmc_l.theta, vmc_l.d_theta, vmc_r.theta, vmc_r.d_theta, GBC486.euler[1], GBC486.gyro[1], chassis_kf_l, chassis_kf_r, chassis_cmd, Off_Ground_Flag)
            
            w_r = 0
            w_l = 0

            # w_r = controller.UR[0]
            # w_l = controller.UL[0]
            # vmc_r.Tp = controller.UR[1]
            # vmc_l.Tp = controller.UL[1]   

            GBC486.wheel_torque = [w_r,w_l]
            GBC486.joint_torque = [vmc_r.torque_set[1],vmc_r.torque_set[0],vmc_l.torque_set[0],vmc_l.torque_set[1]]
            GBC486.actuator_set_torque()

        #键盘控制指令输入,以及打印数据;运行频率低以降低仿真延迟
        if i % t3 == 0:
            cmd = keyboard.get_command()
            # print(GBC486.euler[1])
            # print(vmc_r.phi1,vmc_r.phi4,vmc_l.phi1,vmc_l.phi4)
            # print(GBC486.joint_pos[0],GBC486.joint_pos[1],GBC486.joint_pos[2],GBC486.joint_pos[3])
            print(vmc_r.L0,vmc_r.phi0,vmc_l.L0,vmc_l.phi0)



if __name__ == '__main__':
    main()

