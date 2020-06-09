import numpy as np
from scipy.interpolate import CubicSpline

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.v = 2 #m/s
        self.points = points
        self.t = np.zeros(len(points),)

        if np.shape(self.points) == (3,) or np.shape(self.points) == (1,3):
            pass
        elif np.shape(self.points) != (3,) or np.shape(self.points) != (1,3):
            for i in range(len(self.t)-1):
                self.t[(i+1)] = np.linalg.norm((points[(i+1)]-points[i]))/self.v

            self.point_t = np.zeros(len(points),)
            for i in range(int(len(self.t)-1)):
                self.point_t[(i+1)] = self.point_t[i] + self.t[i+1]

            self.f = CubicSpline(self.point_t,self.points,axis = 0)




    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        if np.shape(self.points) == (3,) or np.shape(self.points) == (1,3):
            x = np.reshape(self.points,(3,))
        elif np.shape(self.points) != (3,) or np.shape(self.points) != (1,3):
            if t > self.point_t[-1]:
                x = self.points[-1]
            else:
                x = self.f(t)



        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
