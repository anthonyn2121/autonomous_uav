import numpy as np
from scipy.spatial.transform import Rotation
import math as m

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        self.Kd = np.diag(np.array([7.5, 7.5, 7.5]))
        self.Kp = np.diag(np.array([23, 23, 23]))
        self.Kr = np.diag(np.array([250, 250, 250]))
        self.Kw = np.diag(np.array([20, 20, 20]))

        self.grav = np.array([[0], [0], [self.mass * self.g]])
        self.b_b = np.array([[0], [0], [1]])  # Quadrotor's axis in its frame
        self.gamma = self.k_drag / self.k_thrust
        self.A = np.array([[1, 1, 1, 1],
                           [0, self.arm_length, 0, -self.arm_length],
                           [-self.arm_length, 0, self.arm_length, 0],
                           [self.gamma, -self.gamma, self.gamma, -self.gamma]])

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        # convert quaternion to rotation matrix
        R = Rotation.from_quat(state['q']).as_matrix()

        # Calculate r_des
        state_x = state['x']
        state_x = state_x.T
        state_v = state['v']
        x_des = flat_output['x']
        x_dot_des = flat_output['x_dot']
        x_ddot_des = flat_output['x_ddot']
        r_des = x_ddot_des+np.matmul(-self.Kd, (state_v-x_dot_des)) + np.matmul(-self.Kp, (state_x-x_des))
        #convert r_des to a (3x1)
        r_des = np.tile(r_des, (2, 1))
        r_des = r_des.T
        r_des = np.delete(r_des, 1, 1)

        # Calc F_des -- total commanded force
        F_des = self.mass*r_des + self.grav

        # Calc u1
        b3 = np.matmul(R,self.b_b)
        u1 = np.matmul(b3.T,F_des)

        # Calc u2
        b3_des = F_des/np.linalg.norm(F_des)
        a_psi  = np.array([ [np.cos(flat_output['yaw'])], [np.sin(flat_output['yaw'])],[0] ])
        b2_des = np.cross(b3_des.T,a_psi.T)/np.linalg.norm(np.cross(b3_des.T,a_psi.T))
        # print('b2_des dim: ', np.shape(b2_des))
        b1_des = np.cross(b2_des,b3_des.T)
        # print('b1_des dim: ', np.shape(b1_des))
        R_des = np.concatenate((b1_des.T, b2_des.T, b3_des), axis=1)
        eR = .5*(np.matmul(R_des.T,R)-np.matmul(R.T,R_des))
        eR = np.array([eR[2, 1], eR[0, 2], eR[1, 0]])
        eR = eR.reshape((3,1))
        w_des = np.zeros((3))
        eW = state['w'] - w_des
        eW = eW.reshape((3,1))
        u2 = np.matmul(self.inertia,(np.matmul(-self.Kr,eR) - np.matmul(self.Kw,eW)))


        #control inputs -Fi
        u = np.array([u1[0], u2[0], u2[1], u2[2]])
        F = np.matmul(np.linalg.inv(self.A),u)

        for i in range(len(F)):
            if F[i] < 0:
                F[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_min
            cmd_motor_speeds[i] = m.sqrt(F[i]/self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max

        # print(cmd_motor_speeds)




        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}

        return control_input
