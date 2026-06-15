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
        self.a11[:] = [38.4256, -30.1087, -0.4442, -19.9394]
        self.a12[:] = [6.6674, -7.1735, 0.3292, -3.2346]
        self.a13[:] = [0.6807, -0.4348, -0.0422, -0.4459]
        self.a14[:] = [2.6600, -1.7650, -0.1517, -1.8285]
        self.a15[:] = [14.5711, -11.8894, 0.3023, 2.2214]
        self.a16[:] = [4.2407, -3.3604, 0.0317, 0.6290]
        self.a21[:] = [47.8249, -38.9643, 1.0068, 5.9356]
        self.a22[:] = [7.1990, -5.6323, 0.0493, 1.7816]
        self.a23[:] = [2.2244, -1.7424, 0.0137, 0.3741]
        self.a24[:] = [8.8892, -6.9328, 0.0395, 1.4988]
        self.a25[:] = [-10.1763, 7.0033, 0.3390, 11.8457]
        self.a26[:] = [-1.2334, 0.7347, 0.0923, 2.0357]
    # Q=diag([20 30 1 1 350 5]);
    # R=diag([2.75 0.75]);
# K矩阵 = 
#   [-18.356772,  -3.434565,  -0.513724,  -1.268516,   2.801690,   0.831265;
#      8.328554,   2.889784,   0.603627,   1.433583,  15.815149,   3.210288]

    def _set_coeffs_leg_low(self):
        self.a11[:] = [38.4256, -30.1087, -0.4442, -19.9394]
        self.a12[:] = [6.6674, -7.1735, 0.3292, -3.2346]
        self.a13[:] = [0.6807, -0.4348, -0.0422, -0.4459]
        self.a14[:] = [2.6600, -1.7650, -0.1517, -1.8285]
        self.a15[:] = [14.5711, -11.8894, 0.3023, 2.2214]
        self.a16[:] = [4.2407, -3.3604, 0.0317, 0.6290]
        self.a21[:] = [47.8249, -38.9643, 1.0068, 5.9356]
        self.a22[:] = [7.1990, -5.6323, 0.0493, 1.7816]
        self.a23[:] = [2.2244, -1.7424, 0.0137, 0.3741]
        self.a24[:] = [8.8892, -6.9328, 0.0395, 1.4988]
        self.a25[:] = [-10.1763, 7.0033, 0.3390, 11.8457]
        self.a26[:] = [-1.2334, 0.7347, 0.0923, 2.0357]
#     Q=diag([12 50 1 1 600 10]);
#     R=diag([2.75 1.25]);
# K矩阵 =
#   [-13.109532,  -4.503420,  -0.592536,  -1.461165,   5.320545,   0.825572;
#      5.977486,   1.976190,   0.165818,   0.439431,  23.858752,   3.209238]
    def _set_coeffs_leg_mid_or_high(self):
        self.a11[:] = [38.4256, -30.1087, -0.4442, -19.9394]
        self.a12[:] = [6.6674, -7.1735, 0.3292, -3.2346]
        self.a13[:] = [0.6807, -0.4348, -0.0422, -0.4459]
        self.a14[:] = [2.6600, -1.7650, -0.1517, -1.8285]
        self.a15[:] = [14.5711, -11.8894, 0.3023, 2.2214]
        self.a16[:] = [4.2407, -3.3604, 0.0317, 0.6290]
        self.a21[:] = [47.8249, -38.9643, 1.0068, 5.9356]
        self.a22[:] = [7.1990, -5.6323, 0.0493, 1.7816]
        self.a23[:] = [2.2244, -1.7424, 0.0137, 0.3741]
        self.a24[:] = [8.8892, -6.9328, 0.0395, 1.4988]
        self.a25[:] = [-10.1763, 7.0033, 0.3390, 11.8457]
        self.a26[:] = [-1.2334, 0.7347, 0.0923, 2.0357]

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
        # if(abs(pitch) < 0.1):
        #     self.obs_L[0] = theta_left + pitch*13.5
        #     self.obs_R[0] = theta_right + pitch*13.5
        # else:
        #    self.obs_L[0] = theta_left 
        #    self.obs_R[0] = theta_right

        self.obs_L[0] = theta_left - 0.2
        self.obs_R[0] = theta_right - 0.2



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
