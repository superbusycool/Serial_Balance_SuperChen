"""
Light-weight Python port of chassis_task.c LQR logic.
API: create LQRController() and call update(...) each control cycle.

Expected minimal attributes on inputs (can be simple objects or dicts):
 - leg: attributes theta, d_theta_lpf, d_phi0, l0
 - ins: attributes pitch (deg), gyro (deg/s), yaw_total_angle (deg)
 - chassis_kf_l / chassis_kf_r: objects with FilteredValue[0]
 - chassis_cmd: attributes ctrl_mode, leg_level, leg_leng_change, vx_set
 - Off_Ground_Flag: boolean/int

Returns: (u_left, u_right) each is a length-2 numpy array [T, Tp]

Constants (RC_DBUS_MAX_VALUE, CHASSIS_V_SET, etc) follow the C implementation.
"""

import numpy as np

DEG2RAD = np.pi / 180.0
RC_DBUS_MAX_VALUE = 660.0
CHASSIS_V_SET = 2.0
X_RATIO = 0.4

# Enums (match C defines)
CHASSIS_RELAX = 0
CHASSIS_RECOVERY = 1
CHASSIS_OPENLOOP = 2

LENGTH_STAY = 0

LEG_LOW = 0
LEG_MID = 1
LEG_HIG = 2


# Small helper types expected by the rest of the codebase.
class ChassisCmd:
        """Simple container for chassis command fields expected by LQR.

        Fields:
            - ctrl_mode: int (e.g. CHASSIS_RECOVERY or CHASSIS_RELAX)
            - leg_length: float (the desired leg length)
            - leg_leng_change: int (LENGTH_STAY...)
            - off_ground_flag: int (0 or 1)
            - vx_set: numeric (rc dbus value, same scale used in LQR)
        """
        def __init__(self, ctrl_mode, leg_length, leg_leng_change, leg_level, off_ground_flag, vx_set, vw_set):
                self.ctrl_mode = ctrl_mode
                self.leg_length = leg_length
                self.leg_leng_change = leg_leng_change
                self.leg_level = leg_level
                self.Off_Ground_Flag = off_ground_flag
                self.vx_set = vx_set
                self.vw_set = vw_set 


class ChassisKF:
        """Minimal placeholder that provides `FilteredValue` list used by LQR.

        The real project has a filter object; for testing a single-element list
        with vx is sufficient.
        """
        def __init__(self, vx=0.0):
                self.FilteredValue = [vx]

class LQRController:
    def __init__(self):
        # polynomial coefficient containers: a11..a26, each is length-4
        self.a11 = np.zeros(4)
        self.a12 = np.zeros(4)
        self.a13 = np.zeros(4)
        self.a14 = np.zeros(4)
        self.a15 = np.zeros(4)
        self.a16 = np.zeros(4)
        self.a21 = np.zeros(4)
        self.a22 = np.zeros(4)
        self.a23 = np.zeros(4)
        self.a24 = np.zeros(4)
        self.a25 = np.zeros(4)
        self.a26 = np.zeros(4)

        # internal integrators for X (index 2 in state)
        self.x_int = [0.0, 0.0]  # left, right

        # X matrix  
        self.obs_L = np.zeros(6)
        self.obs_R = np.zeros(6)

        # K matrix
        self.K = np.zeros((2,6))

        # ERROR matrix
        self.err_L = np.zeros(6)
        self.err_R = np.zeros(6)

        # U matrix
        self.UL = np.zeros((2,1))
        self.UR = np.zeros((2,1))

        # yaw_target kept for recovery case
        self.yaw_target = 0.0


#     Q=diag([12 60 1 1 1 1]);
#     R=diag([2.75 1.25]);
#    K矩阵 =
# K矩阵 = 
#   [-16.782528,  -4.868173,  -0.561136,  -1.408121,   0.220532,   0.289484;
#      5.787546,   2.209635,   0.327101,   0.762465,   0.832299,   0.943767]
    def _set_coeffs_recovery(self):
        self.a11[:] = [26.3208, -20.4101, -0.2760, -19.3201]
        self.a12[:] = [5.1980, -5.1870, 0.1669, -6.1456]
        self.a13[:] = [0.3583, -0.2316, -0.0212, -0.7167]
        self.a14[:] = [0.2569, -0.1858, -0.0176, -1.7778]
        self.a15[:] = [1.3294, -1.0226, -0.0023, 0.2366]
        self.a16[:] = [1.7978, -1.3726, -0.0083, 0.3177]
        self.a21[:] = [23.1947, -18.5302, 0.2922, 4.6258]
        self.a22[:] = [9.5327, -7.2793, -0.0399, 1.8596]
        self.a23[:] = [1.5730, -1.2099, -0.0027, 0.2799]
        self.a24[:] = [3.6568, -2.7646, -0.0313, 0.6373]
        self.a25[:] = [-0.4240, 0.2740, 0.0251, 0.8481]
        self.a26[:] = [-0.5901, 0.3781, 0.0371, 0.9628]
    # Q=diag([20 30 1 1 350 5]);
    # R=diag([2.75 0.75]);
# K矩阵 = 
#   [-18.356772,  -3.434565,  -0.513724,  -1.268516,   2.801690,   0.831265;
#      8.328554,   2.889784,   0.603627,   1.433583,  15.815149,   3.210288]

    def _set_coeffs_leg_low(self):
        self.a11[:] = [37.9648, -28.1465, -0.7507, -18.0382]
        self.a12[:] = [6.5894, -6.2072, 0.1448, -3.3936]
        self.a13[:] = [1.0038, -0.6491, -0.0551, -0.5027]
        self.a14[:] = [2.2100, -1.4702, -0.1157, -1.2445]
        self.a15[:] = [19.1009, -15.7070, 0.4953, 2.8901]
        self.a16[:] = [5.5885, -4.4509, 0.0668, 0.8635]
        self.a21[:] = [66.4517, -55.2397, 1.9697, 8.6175]
        self.a22[:] = [14.7984, -11.9139, 0.2443, 2.9697]
        self.a23[:] = [3.7870, -3.0191, 0.0527, 0.6248]
        self.a24[:] = [9.0557, -7.1614, 0.0962, 1.4865]
        self.a25[:] = [-18.4241, 12.5212, 0.6415, 15.6442]
        self.a26[:] = [-3.2745, 2.0910, 0.1764, 3.1750]
#     Q=diag([12 50 1 1 600 10]);
#     R=diag([2.75 1.25]);
# K矩阵 =
#   [-13.109532,  -4.503420,  -0.592536,  -1.461165,   5.320545,   0.825572;
#      5.977486,   1.976190,   0.165818,   0.439431,  23.858752,   3.209238]
    def _set_coeffs_leg_mid_or_high(self):
        self.a11[:] = [24.9617, -20.0250, 0.0845, -12.9427]
        self.a12[:] = [5.0679, -4.8726, 0.1285, -4.4726]
        self.a13[:] = [0.1261, -0.0848, -0.0039, -0.5914]
        self.a14[:] = [-0.2935, 0.1847, 0.0132, -1.4640]
        self.a15[:] = [33.3435, -25.9189, 0.1794, 5.5285]
        self.a16[:] = [4.5320, -3.5195, 0.0225, 0.8540]
        self.a21[:] = [13.3545, -11.4234, 0.6248, 6.0159]
        self.a22[:] = [11.7852, -9.2040, 0.0920, 2.0472]
        self.a23[:] = [1.1137, -0.8883, 0.0209, 0.1715]
        self.a24[:] = [3.0896, -2.3823, 0.0153, 0.4586]
        self.a25[:] = [-14.5507, 9.7773, 0.6100, 23.7145]
        self.a26[:] = [-2.1654, 1.4780, 0.0803, 3.1886]

    def _poly_eval(self, a, l0_pow3,l0_pow2, l0):
        # a: array [a0,a1,a2,a3] corresponds to coeffs used in C as [0]*l0^3 + [1]*l0^2 + [2]*l0 + [3]

        return a[0]*l0_pow3 + a[1]*l0_pow2 + a[2]*l0 + a[3]

    def update_K(self, chassis_cmd, leg_left_l0_pow2=None, leg_left_l0_pow3=None):
        # choose coefficient set
        if chassis_cmd.ctrl_mode == CHASSIS_RECOVERY:
            self._set_coeffs_recovery()
        else:
            if chassis_cmd.leg_level == LEG_LOW:
                self._set_coeffs_leg_low()
            else:
                self._set_coeffs_leg_mid_or_high()

        l0 = chassis_cmd.leg_length
        l0_pow3 = l0**3
        l0_pow2 = l0**2
        # Wheel shut condition: only K21,K22 kept
        if chassis_cmd.leg_leng_change == LENGTH_STAY and chassis_cmd.Off_Ground_Flag:
            # first row zeros
            self.K[0,:] = 0.0
            self.K[1,0] = self._poly_eval(self.a21,l0_pow3,l0_pow2, l0)
            self.K[1,1] = self._poly_eval(self.a22, l0_pow3,l0_pow2, l0)
            self.K[1,2] = 0.0
            self.K[1,3] = 0.0
            self.K[1,4] = 0.0
            self.K[1,5] = 0.0
        else:
            self.K[0,0] = self._poly_eval(self.a11, l0_pow3,l0_pow2, l0)
            self.K[0,1] = self._poly_eval(self.a12, l0_pow3,l0_pow2, l0)
            self.K[0,2] = self._poly_eval(self.a13, l0_pow3,l0_pow2, l0)
            self.K[0,3] = self._poly_eval(self.a14, l0_pow3,l0_pow2, l0)
            self.K[0,4] = self._poly_eval(self.a15, l0_pow3,l0_pow2, l0)
            self.K[0,5] = self._poly_eval(self.a16, l0_pow3,l0_pow2, l0)
            self.K[1,0] = self._poly_eval(self.a21, l0_pow3,l0_pow2, l0)
            self.K[1,1] = self._poly_eval(self.a22, l0_pow3,l0_pow2, l0)
            self.K[1,2] = self._poly_eval(self.a23, l0_pow3,l0_pow2, l0)
            self.K[1,3] = self._poly_eval(self.a24, l0_pow3,l0_pow2, l0)
            self.K[1,4] = self._poly_eval(self.a25, l0_pow3,l0_pow2, l0)
            self.K[1,5] = self._poly_eval(self.a26, l0_pow3,l0_pow2, l0)

    def update(self,l0_left,l0_right,theta_left, theta_left_dot,  theta_right, theta_right_dot, pitch,pitch_gyro, chassis_kf_l, chassis_kf_r, chassis_cmd):
        """Compute LQR outputs for left and right legs.
        leg_left/right: objects with attributes theta, d_theta_lpf, d_phi0, l0
        ins: object with pitch (deg), gyro (deg/s), yaw_total_angle (deg)
        chassis_kf_l/r: objects with FilteredValue list/array (index 0 is vx)
        chassis_cmd: object with ctrl_mode, leg_level, leg_leng_change, off_ground_flag, vx_set
        """
        # update filters and K

        # update K
        self.update_K(chassis_cmd)


        # d_theta
        self.obs_L[1] = theta_left_dot
        self.obs_R[1] = theta_right_dot

        # vx
        self.obs_L[3] = chassis_kf_l
        self.obs_R[3] = chassis_kf_r

        # pitch and gyro (C uses negative)
        self.obs_L[4] = -pitch 
        self.obs_L[5] = -pitch_gyro 
        self.obs_R[4] = -pitch 
        self.obs_R[5] = -pitch_gyro 

        # # theta with compensation
        # if chassis_cmd.ctrl_mode == CHASSIS_RECOVERY:
        #     # compensation value from C
        #     theta_comp = -0.16
        #     obs_L[0] = theta_left + theta_comp
        #     obs_R[0] = theta_right + theta_comp
        # else:
        #     if chassis_cmd.leg_level == LEG_LOW:
        #         theta_comp = -0.115
        #     elif chassis_cmd.leg_level == LEG_MID:
        #         theta_comp = -0.105
        #     else:
        #         theta_comp = -0.05
        #     obs_L[0] = theta_left + theta_comp
        #     obs_R[0] = theta_right + theta_comp

        self.obs_L[0] = theta_left - 0.23
        self.obs_R[0] = theta_right- 0.23


        self.obs_L[2] = 0
        self.obs_R[2] = 0

        # build ref vectors
        ref = np.zeros(6)
        ref[2] = 0.0
        ref[3] = chassis_cmd.vx_set

        # ref[2] = 0.0
        # ref[3] = 0.0  #静止情况下

        # compute error = ref - obs
        self.err_L = ref - self.obs_L
        self.err_R = ref - self.obs_R

        # compute outputs: u = -K * err  (C multiplies MatLQRNegK by Err)
        # return as 2-element arrays [T, Tp]
        self.UL = - self.K.dot(self.err_L)
        self.UR = - self.K.dot(self.err_R)


# Example minimal usage (to be used by the main simulation loop):
# controller = LQRController()
# uL, uR = controller.update(leg_left, leg_right, ins, chassis_kf_l, chassis_kf_r, chassis_cmd, Off_Ground_Flag)
# then apply uL[0],uL[1] etc to actuators
