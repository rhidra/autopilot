import numpy as np
from motion_primitive import MotionPrimitive


class TrajectoryError(Exception):
    pass


class MotionPrimitiveLibrary:
    def __init__(self, delta_yaw=31, delta_norm=10, delta_z=5, tf=1):
        self.delta_yaw = delta_yaw
        self.delta_norm = delta_norm
        self.delta_z = delta_z
        self.tf = tf
        self.trajs = []

    def generate_traj_library(self, pos0, vel0, acc0):
        norm0 = np.sqrt(vel0[0]*vel0[0] + vel0[1]*vel0[1])
        yaw0 = np.arctan2(vel0[1], vel0[0])

        self.trajs = []
        for zf in np.linspace(pos0[2] - 1, pos0[2] + 1, self.delta_z):
            for yaw in np.linspace(yaw0 - np.pi*.45, yaw0 + np.pi*.45, self.delta_yaw):
                for norm in np.linspace(np.clip(norm0 - 1, .1, 1e5), norm0 + 1, self.delta_norm):
                    self.trajs.append(self.generate_traj(pos0, vel0, acc0, [norm * np.cos(yaw), norm * np.sin(yaw), 0], zf))

    def generate_traj(self, pos0, vel0, acc0, velf, zf):
        traj = MotionPrimitive(pos0, vel0, acc0, [0, 0, -9.81])
        traj.set_goal_position([None, None, zf])
        traj.set_goal_velocity(velf)
        traj.set_goal_acceleration([0, 0, 0])
        traj.generate(self.tf)
        return traj
    
    def rank_trajectories(self, goal_point, goal_direction, get_point_edt):
        for traj in self.trajs:
            traj.compute_cost(goal_point, goal_direction, get_point_edt)
    
    def get_best_traj(self):
        traj = min(self.trajs, key=lambda t: t._cost)
        if traj._cost > 1000:
            raise TrajectoryError()
        return traj 

