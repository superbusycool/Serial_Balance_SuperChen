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
CHASSIS_RECOVERY = 1
CHASSIS_RELAX = 2
LENGTH_STAY = 0
LEG_LOW = 0
LEG_MID = 1
LEG_HIG = 2


# Small helper types expected by the rest of the codebase.
class ChassisCmd:
        """Simple container for chassis command fields expected by LQR.

        Fields:
            - ctrl_mode: int (e.g. CHASSIS_RECOVERY or CHASSIS_RELAX)
            - leg_level: int (LEG_LOW/LEG_MID/LEG_HIG)
            - leg_leng_change: int (LENGTH_STAY...)
            - vx_set: numeric (rc dbus value, same scale used in LQR)
        """
        def __init__(self, ctrl_mode=CHASSIS_RELAX, leg_level=LEG_MID, leg_leng_change=LENGTH_STAY, vx_set=0.0):
                self.ctrl_mode = ctrl_mode
                self.leg_level = leg_level
                self.leg_leng_change = leg_leng_change
                self.vx_set = vx_set


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

        # K matrix
        self.K = np.zeros((2,6))

        # U matrix
        self.UL = np.zeros((2,1))
        self.UR = np.zeros((2,1))

        # yaw_target kept for recovery case
        self.yaw_target = 0.0


    def _set_coeffs_recovery(self):
        self.a11[:] = [29.3085, -22.8226, -0.1941, -12.7796]
        self.a12[:] = [5.5615, -5.2622, 0.1322, -4.7719]
        self.a13[:] = [0.1723, -0.1044, -0.0140, -0.5834]
        self.a14[:] = [0.2742, -0.1499, -0.0440, -1.4040]
        self.a15[:] = [10.8752, -8.5748, 0.0820, 2.0460]
        self.a16[:] = [1.8945, -1.4936, 0.0153, 0.4600]
        self.a21[:] = [-12.0512, 9.0112, 0.0279, 3.0024]
        self.a22[:] = [11.9565, -9.4224, 0.0737, 2.3181]
        self.a23[:] = [-1.2817, 0.9307, 0.0201, -0.2228]
        self.a24[:] = [-2.4378, 1.7326, 0.0558, -0.4529]
        self.a25[:] = [-3.2360, 2.6894, -0.0966, 5.9674]
        self.a26[:] = [-0.5576, 0.4297, -0.0010, 1.2598]

    def _set_coeffs_leg_low(self):
        self.a11[:] = [23.8179, -19.3163, 0.1346, -11.9056]
        self.a12[:] = [4.5713, -4.5667, 0.1545, -3.5534]
        self.a13[:] = [0.1093, -0.0739, -0.0030, -0.5932]
        self.a14[:] = [-0.2616, 0.1487, 0.0164, -1.4019]
        self.a15[:] = [26.8530, -20.8822, 0.1433, 4.4775]
        self.a16[:] = [3.5397, -2.7494, 0.0168, 0.6913]
        self.a21[:] = [10.4188, -9.0329, 0.5544, 5.5825]
        self.a22[:] = [9.0860, -7.1064, 0.0756, 1.6060]
        self.a23[:] = [1.0448, -0.8387, 0.0228, 0.1577]
        self.a24[:] = [2.7610, -2.1434, 0.0221, 0.4032]
        self.a25[:] = [-12.0060, 8.1418, 0.4706, 18.8611]
        self.a26[:] = [-1.7684, 1.2237, 0.0582, 2.4453]

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

    def _poly_eval(self, a, l0):
        # a: array [a0,a1,a2,a3] corresponds to coeffs used in C as [0]*l0^3 + [1]*l0^2 + [2]*l0 + [3]
        l0_pow3 = l0**3
        l0_pow2 = l0**2
        return a[0]*l0_pow3 + a[1]*l0_pow2 + a[2]*l0 + a[3]

    def update_K(self, leg_l0_average, chassis_cmd, Off_Ground_Flag, leg_left_l0_pow2=None, leg_left_l0_pow3=None):
        # choose coefficient set
        if chassis_cmd.ctrl_mode == CHASSIS_RECOVERY:
            self._set_coeffs_recovery()
        else:
            if chassis_cmd.leg_level == LEG_LOW:
                self._set_coeffs_leg_low()
            else:
                self._set_coeffs_leg_mid_or_high()

        l0 = leg_l0_average
        # Wheel shut condition: only K21,K22 kept
        if chassis_cmd.leg_leng_change == LENGTH_STAY and Off_Ground_Flag:
            # first row zeros
            self.K[0,:] = 0.0
            self.K[1,0] = self._poly_eval(self.a21, l0)
            self.K[1,1] = self._poly_eval(self.a22, l0)
            self.K[1,2] = 0.0
            self.K[1,3] = 0.0
            self.K[1,4] = 0.0
            self.K[1,5] = 0.0
        else:
            self.K[0,0] = self._poly_eval(self.a11, l0)
            self.K[0,1] = self._poly_eval(self.a12, l0)
            self.K[0,2] = self._poly_eval(self.a13, l0)
            self.K[0,3] = self._poly_eval(self.a14, l0)
            self.K[0,4] = self._poly_eval(self.a15, l0)
            self.K[0,5] = self._poly_eval(self.a16, l0)
            self.K[1,0] = self._poly_eval(self.a21, l0)
            self.K[1,1] = self._poly_eval(self.a22, l0)
            self.K[1,2] = self._poly_eval(self.a23, l0)
            self.K[1,3] = self._poly_eval(self.a24, l0)
            self.K[1,4] = self._poly_eval(self.a25, l0)
            self.K[1,5] = self._poly_eval(self.a26, l0)

    def update(self,l0_left,l0_right,theta_left, theta_left_dot,  theta_right, theta_right_dot, pitch,pitch_gyro, chassis_kf_l, chassis_kf_r, chassis_cmd, Off_Ground_Flag):
        """Compute LQR outputs for left and right legs.
        leg_left/right: objects with attributes theta, d_theta_lpf, d_phi0, l0
        ins: object with pitch (deg), gyro (deg/s), yaw_total_angle (deg)
        chassis_kf_l/r: objects with FilteredValue list/array (index 0 is vx)
        chassis_cmd: object with ctrl_mode, leg_level, leg_leng_change, vx_set
        Off_Ground_Flag: bool/int
        """
        # update filters and K
        chassis_vx_filter = 0.5 * (chassis_kf_l.FilteredValue[0] + chassis_kf_r.FilteredValue[0])

        # leg average
        l0_average = 0.5 * (l0_left + l0_right)

        # update K
        self.update_K(l0_average, chassis_cmd, Off_Ground_Flag)

        # build obs vectors
        obs_L = np.zeros(6)
        obs_R = np.zeros(6)

        # d_theta
        obs_L[1] = theta_left_dot
        obs_R[1] = theta_right_dot

        # vx
        obs_L[3] = chassis_vx_filter
        obs_R[3] = chassis_vx_filter

        # pitch and gyro (C uses negative)
        obs_L[4] = -pitch
        obs_L[5] = -pitch_gyro 
        obs_R[4] = -pitch 
        obs_R[5] = -pitch_gyro 

        # theta with compensation
        if chassis_cmd.ctrl_mode == CHASSIS_RECOVERY:
            # compensation value from C
            theta_comp = -0.16
            obs_L[0] = theta_left + theta_comp
            obs_R[0] = theta_right + theta_comp
        else:
            if chassis_cmd.leg_level == LEG_LOW:
                theta_comp = -0.115
            elif chassis_cmd.leg_level == LEG_MID:
                theta_comp = -0.105
            else:
                theta_comp = -0.05
            obs_L[0] = theta_left + theta_comp
            obs_R[0] = theta_right + theta_comp

        # displacement integrator (index 2)
        # compute speed error: err = ref[3] - obs[3]
        ref_vx = (chassis_cmd.vx_set / RC_DBUS_MAX_VALUE) * CHASSIS_V_SET
        err_speed_L = ref_vx - obs_L[3]
        err_speed_R = ref_vx - obs_R[3]

        # update integrator condition similar to C
        # when both legs' speed error small and average ref small, integrate
        # we need previous error values; approximate by checking magnitude
        if (abs(err_speed_L) < 0.01) and (abs(err_speed_R) < 0.01) and (0.5 * (ref_vx + ref_vx) < 0.01):
            self.x_int[0] -= err_speed_L * X_RATIO
            self.x_int[1] -= err_speed_R * X_RATIO
        else:
            self.x_int[0] = 0.0
            self.x_int[1] = 0.0

        obs_L[2] = self.x_int[0]
        obs_R[2] = self.x_int[1]

        # build ref vectors
        ref = np.zeros(6)
        ref[2] = 0.0
        ref[3] = ref_vx

        # compute error = ref - obs
        err_L = ref - obs_L
        err_R = ref - obs_R

        # compute outputs: u = -K * err  (C multiplies MatLQRNegK by Err)
        # return as 2-element arrays [T, Tp]
        self.UL = - self.K.dot(err_L)
        self.UR = - self.K.dot(err_R)


# Example minimal usage (to be used by the main simulation loop):
# controller = LQRController()
# uL, uR = controller.update(leg_left, leg_right, ins, chassis_kf_l, chassis_kf_r, chassis_cmd, Off_Ground_Flag)
# then apply uL[0],uL[1] etc to actuators
