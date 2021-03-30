import numpy as np, time
from motion_primitive import MotionPrimitive


class TrajectoryError(Exception):
    def __init__(self, traj):
        self.traj = traj

FEASIBLE_TRAJ_THRESHOLD = 1000

class MotionPrimitiveLibrary:
    def __init__(self, delta_yaw=21, delta_norm=15, tf=1):
        self.delta_yaw = delta_yaw
        self.delta_norm = delta_norm
        self.tf = tf
        self.trajs = []

    def generate_traj_library(self, pos0, vel0, acc0, zf=None):
        if zf is None:
            zf = pos0[2]
        norm0 = np.linalg.norm(vel0)
        yaw0 = np.arctan2(vel0[1], vel0[0])

        self.trajs = []

        # Final Z range
        # We sample regularly over z if necessary between z0 and zf
        Nz = np.clip(np.int(np.abs(pos0[2] - zf) / .5), 2, 10)
        for z in np.linspace(pos0[2], zf, Nz)[1:]:
            # Yaw range
            for yaw in np.linspace(yaw0 - np.pi*.5, yaw0 + np.pi*.5, self.delta_yaw):
                # Velocity norm range
                for norm in np.linspace(np.clip(norm0 - 4/self.tf, .1, 1e5), norm0 + .5/self.tf, self.delta_norm):
                    self.trajs.append(self.generate_traj(pos0, vel0, acc0, [norm * np.cos(yaw), norm * np.sin(yaw), 0], z))

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
        if traj._cost > FEASIBLE_TRAJ_THRESHOLD:
            raise TrajectoryError(traj)
        return traj 

