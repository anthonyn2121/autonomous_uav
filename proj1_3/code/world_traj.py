import numpy as np

from proj1_3.code.graph_search import graph_search
from scipy.interpolate import CubicSpline, interp1d


class WorldTraj(object):
    """

    """

    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.2, 0.2, 0.2])
        self.margin = 0.3

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1, 3))  # shape=(n_pts,3)
        # self.points = self.path

        self.points = np.array(self.path[0])
        for i in range(self.path.shape[0]):
            if i != 0 and i != (self.path.shape[0] - 1):
                segment_1 = (self.path[i] - self.path[i-1])
                segment_2 = (self.path[i+1] - self.path[i-1])
                linear_segment = np.round(np.cross(segment_1, segment_2),5)
                if np.array_equal(linear_segment, [0, 0, 0]):
                    pass
                else:
                    self.points = np.vstack((self.points, self.path[i]))
                    print('added point')

        self.points = np.vstack((self.points, self.path[-1]))
        print(self.points.shape)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        self.v = 2.5  # m/s
        self.t = np.zeros(len(self.points), )
        for i in range(len(self.t) - 1):
            self.t[(i + 1)] = np.linalg.norm((self.points[(i + 1)] - self.points[i])) / self.v

        self.point_t = np.zeros(len(self.points), )
        for i in range(int(len(self.t) - 1)):
            self.point_t[(i + 1)] = self.point_t[i] + self.t[i + 1]

        # self.f = CubicSpline(self.point_t, self.points, axis=0)
        self.f = interp1d(self.point_t, self.points, axis= 0)

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

        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE

        # from waypoint_traj
        if np.shape(self.points) == (3,) or np.shape(self.points) == (1, 3):
            x = np.reshape(self.points, (3,))
        elif np.shape(self.points) != (3,) or np.shape(self.points) != (1, 3):
            if t > self.point_t[-1]:
                x = self.points[-1]
            else:
                x = self.f(t)

        # if np.shape(self.points) == (3,) or np.shape(self.points) == (1,3):
        #     x = np.reshape(self.points, (3,))
        # elif np.shape(self.points) != (3,) or np.shape(self.points) != (1,3):
        #     if t >= self.t[-1]:
        #         x = self.points[-1]
        #     elif t == self.t[0]:
        #         x = self.points[0]
        #     else:
        #         for i in range(len(self.t)-1):
        #             if self.t[i] < t <= self.t[i+1]:
        #                 x_dot = self.v
        #                 x = x_dot*(t-self.t[i]) + self.points[i]
        #


        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}
        return flat_output
