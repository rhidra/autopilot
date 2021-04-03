import numpy as np, time, scipy.optimize
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
        self.pos0, self.vel0, self.acc0 = np.zeros(3), np.zeros(3), np.zeros(3)
        self.goal_point, self.goal_direction = np.zeros(3), np.zeros(3)
        self.edt_function = None

    def set_init_state(self, pos0, vel0, acc0):
        self.pos0, self.vel0, self.acc0 = pos0, vel0, acc0
    
    def set_local_goal(self, goal_point, goal_direction):
        self.goal_point, self.goal_direction = goal_point, goal_direction

    def set_edt_function(self, edt):
        self.edt_function = edt

    def optimize(self):
        norm0 = np.linalg.norm(self.vel0)
        yaw0 = np.arctan2(self.vel0[1], self.vel0[0])

        # Optimize with the norm constant, along the yaw values
        # We use the Brent method to optimize
        f = self.build_cost_function(norm=norm0, z=self.pos0[2])
        yaw = scipy.optimize.brent(f, brack=(yaw0 - np.pi * .5, yaw0 + np.pi * .5), tol=1e-2)
        f = self.build_cost_function(yaw=yaw, z=self.pos0[2])
        norm = scipy.optimize.brent(f, brack=(np.clip(norm0 - 4/self.tf, .1, 1e5), norm0 + .5/self.tf), tol=1e-2)
        norm = min(norm, norm0 + .5/self.tf)
        
        traj = self.generate_traj(self.pos0, self.vel0, self.acc0, norm, yaw, self.pos0[2])
        traj.compute_cost(self.goal_point, self.goal_direction, self.edt_function)
        if traj._cost > FEASIBLE_TRAJ_THRESHOLD:
            raise TrajectoryError(traj)
        return traj


    # Generate a trajectory function cost function, by 2 of the scalar parameters
    def build_cost_function(self, norm=None, yaw=None, z=None):
        def cost(norm, yaw, z):
            traj = self.generate_traj(self.pos0, self.vel0, self.acc0, norm, yaw, z)
            traj.compute_cost(self.goal_point, self.goal_direction, self.edt_function)
            self.trajs.append(traj)
            return traj._cost

        if norm is not None and yaw is not None:
            def f(z):
                print('z=', z)
                return cost(norm, yaw, z)
        elif norm is not None:
            def f(yaw):
                print('yaw=', yaw)
                return cost(norm, yaw, z)
        else:
            def f(norm):
                print('norm=', norm)
                return cost(norm, yaw, z)
        return f

    def generate_traj(self, pos0, vel0, acc0, norm_velf, yawf, zf):
        traj = MotionPrimitive(pos0, vel0, acc0, [0, 0, -9.81])
        traj.set_goal_position([None, None, zf])
        traj.set_goal_velocity([norm_velf * np.cos(yawf), norm_velf * np.sin(yawf), 0])
        traj.set_goal_acceleration([0, 0, 0])
        traj.generate(self.tf)
        return traj
    