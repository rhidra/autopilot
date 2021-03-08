"""

SYNOPSIS

    Rapid trajectory generation for quadrocopters

DESCRIPTION
    
    An implementation of the algorithm described in the paper "Rapid 
    quadrocopter trajectory generation". Generates trajectories from an 
    arbitrary initial state, to a final state described by (any combination 
    of) position, velocity, and acceleration.
    
    Please refer to the paper for more information.

EXAMPLES

    Please see attached `demo.py` scripts for an example on how to use the 
    trajectory generator. `demo.py` will generate a trajectory with given 
    constraints, and return whether it passes feasibility tests. Then, some 
    plots are generated to visualise the resulting trajectory.
    
AUTHOR
    
    Mark W. Mueller <mwm@mwm.im>

LICENSE

    Copyright 2014 by Mark W. Mueller <mwm@mwm.im>

    This code is free software: you can redistribute
    it and/or modify it under the terms of the GNU General Public
    License as published by the Free Software Foundation, either
    version 3 of the License, or (at your option) any later version.

    This code is distributed in the hope that it will
    be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
    of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the code.  If not, see <http://www.gnu.org/licenses/>.
    
VERSION 

    0.1

"""

import numpy as np, rospy
from single_axis_trajectory import SingleAxisTrajectory
from autopilot.msg import MotionPrimitive as MotionPrimitiveMsg


class StateFeasibilityResult:
    """An enumeration of the possible outcomes for the state feasiblity test.

    The result is either feasible (0), or infeasible (1).
    """
    Feasible, Infeasible = range(2)
    
    @classmethod
    def to_string(cls,ifr):
        """Return the name of the result."""
        if   ifr==StateFeasibilityResult.Feasible:
            return "Feasible"
        elif ifr==StateFeasibilityResult.Infeasible:
            return "Infeasible"
        return "Unknown"
            

def buildMotionPrimitiveFromMsg(msg):
    pos0 = np.array([msg.pos0.x, msg.pos0.y, msg.pos0.z])
    vel0 = np.array([msg.vel0.x, msg.vel0.y, msg.vel0.z])
    acc0 = np.array([msg.acc0.x, msg.acc0.y, msg.acc0.z])
    gravity = np.array([msg.gravity.x, msg.gravity.y, msg.gravity.z])
    p = MotionPrimitive(pos0, vel0, acc0, gravity)
    p._tf = msg.tf
    p._axis[0]._a, p._axis[1]._a, p._axis[2]._a = msg.alpha.x, msg.alpha.y, msg.alpha.z
    p._axis[0]._b, p._axis[1]._b, p._axis[2]._b = msg.beta.x, msg.beta.y, msg.beta.z
    p._axis[0]._g, p._axis[1]._g, p._axis[2]._g = msg.gamma.x, msg.gamma.y, msg.gamma.z
    return p


class MotionPrimitive:
    """Rapid quadrocopter trajectory generator.

    A quadrocopter state interception trajectory. The trajectory starts at a
    state defined by the vehicle's position, velocity, and acceleration. The
    acceleration can be calculated directly from the quadrocopter's attitude
    and thrust value. The trajectory duration is fixed, and given by the user.

    The trajectory goal state can include any combination of components from
    the quadrocopter's position, velocity, and acceleration. The acceleration
    allows to encode the direction of the quadrocopter's thrust at the end time.

    The trajectories are generated without consideration for any constraints,
    and are optimal with respect to the integral of the jerk squared (which is
    equivalent to an upper bound on a product of the inputs).

    The trajectories can then be tested with respect to input constraints
    (thrust/body rates) with an efficient, recursive algorithm. Whether linear
    combinations of states along the trajectory remain within some bounds can
    also be tested efficiently.

		For more information, please see the publication 'A computationally 
		efficient motion primitive for quadrocopter trajectory generation', 
		avaialable here: http://www.mwm.im/research/publications/

    NOTE: in the publication, axes are 1-indexed, while here they are
    zero-indexed.

    """

    def __init__(self, pos0, vel0, acc0, gravity):
        """Initialise the trajectory.

        Initialise the trajectory with the initial quadrocopter state, and the
        orientation of gravity for this problem.

        The orientation of gravity is required for the feasibility tests.

        Args:
          pos0 (array(3)): Initial position
          vel0 (array(3)): Initial velocity
          acc0 (array(3)): Initial acceleration
          gravity (array(3)): The acceleration due to gravity, in the frame of 
              the trajectories (e.g. [0,0,-9.81] for an East-North-Up frame).

        """

        self._axis = [SingleAxisTrajectory(pos0[i],vel0[i],acc0[i]) for i in range(3)]
        self._grav = gravity
        self._tf = None
        self._cost = np.inf
        self.reset()

    def toMsg(self):
        msg = MotionPrimitiveMsg()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pos0.x, msg.pos0.y, msg.pos0.z = self._axis[0]._p0, self._axis[1]._p0, self._axis[2]._p0
        msg.vel0.x, msg.vel0.y, msg.vel0.z = self._axis[0]._v0, self._axis[1]._v0, self._axis[2]._v0
        msg.acc0.x, msg.acc0.y, msg.acc0.z = self._axis[0]._a0, self._axis[1]._a0, self._axis[2]._a0
        msg.alpha.x, msg.alpha.y, msg.alpha.z = self._axis[0]._a, self._axis[1]._a, self._axis[2]._a
        msg.beta.x, msg.beta.y, msg.beta.z = self._axis[0]._b, self._axis[1]._b, self._axis[2]._b
        msg.gamma.x, msg.gamma.y, msg.gamma.z = self._axis[0]._g, self._axis[1]._g, self._axis[2]._g
        msg.gravity.x, msg.gravity.y, msg.gravity.z = self._grav[0], self._grav[1], self._grav[2]
        msg.tf = self._tf
        return msg

    def set_goal_position(self, pos):
        """ Define the goal end position.

        Define the end position for all three axes. To leave components free, 
        list the end state as ``None`` in the argument, or use the function
        `set_goal_position_in_axis`.

        """
        for i in range(3):
            if pos[i] is None:
                continue
            self.set_goal_position_in_axis(i,pos[i])

    def set_goal_velocity(self, vel):
        """ Define the goal end velocity.

        Define the end velocity for all three axes. To leave components free, 
        list the end state as ``None`` in the argument, or use the function
        `set_goal_velocity_in_axis`.

        """
        for i in range(3):
            if vel[i] is None:
                continue
            self.set_goal_velocity_in_axis(i,vel[i])

    def set_goal_acceleration(self, acc):
        """ Define the goal end acceleration.

        Define the end acceleration for all three axes. To leave components
        free, list the end state as ``None`` in the argument, or use the
        function `set_goal_acceleration_in_axis`.

        """
        for i in range(3):
            if acc[i] is None:
                continue
            self.set_goal_acceleration_in_axis(i,acc[i])

    def set_goal_position_in_axis(self, axNum, pos):
        """ Define the goal end position in axis `axNum`."""
        self._axis[axNum].set_goal_position(pos)

    def set_goal_velocity_in_axis(self, axNum, vel):
        """ Define the goal end velocity in axis `axNum`."""
        self._axis[axNum].set_goal_velocity(vel)

    def set_goal_acceleration_in_axis(self, axNum, acc):
        """ Define the goal end acceleration in axis `axNum`."""
        self._axis[axNum].set_goal_acceleration(acc)

    def reset(self):
        """ Reset the trajectory generator.

        Removes all goal states, and resets the cost. Use this if you want to
        try multiple trajectories from one initial state.

        """
        for i in range(3):
            self._axis[i].reset()

    def generate(self, timeToGo):
        """ Calculate a trajectory of duration `timeToGo`.

        Calculates a trajectory of duration `timeToGo`, with the problem data
        defined so far. If something (e.g. goal position) has not been defined,
        it is assumed to be left free. 

        """
        self._tf = timeToGo
        for i in range(3):
            self._axis[i].generate(self._tf)

    def check_input_feasibility(self, fminAllowed, fmaxAllowed, wmaxAllowed,
                                minTimeSection):
        """ Run recursive input feasibility test on trajectory.

        Attempts to prove/disprove the feasibility of the trajectory with
        respect to input constraints. The result is one of three outcomes:
        (i):   the trajectory is definitely input feasible
        (ii):  the trajectory is definitely input infeasible
        (iii): input feasibility could not be determined

        If the feasibility is indeterminable, this should be treated as
        infeasible.

        Args:
            fminAllowed (float): minimum thrust allowed. [m/s**2]
            fmaxAllowed (float): maximum thrust allowed. [m/s**2]
            wmaxAllowed (float): maximum body rates allowed. [rad/s]
            minTimeSection (float): minimum time interval to be tested during
                the recursion. [s]

        Returns:
            An enumeration, of type InputFeasibilityResult.

        """
        
        return self._check_input_feasibility_section(fminAllowed, fmaxAllowed,
                                    wmaxAllowed, minTimeSection, 0, self._tf)

    def _check_input_feasibility_section(self, fminAllowed, fmaxAllowed, 
                             wmaxAllowed, minTimeSection, t1, t2):
        """Recursive test used by `check_input_feasibility`.

        Returns:
            An enumeration, of type InputFeasibilityResult.

        """

        if (t2-t1)<minTimeSection:
            return InputFeasibilityResult.Indeterminable

        #test the acceleration at the two limits:
        if max(self.get_thrust(t1), self.get_thrust(t2)) > fmaxAllowed:
            return InputFeasibilityResult.InfeasibleThrustHigh
        if min(self.get_thrust(t1), self.get_thrust(t2)) < fminAllowed:
            return InputFeasibilityResult.InfeasibleThrustLow

        fminSqr = 0
        fmaxSqr = 0
        jmaxSqr = 0

        #Test the limits of the box we're putting around the trajectory:
        for i in range(3):
            amin, amax = self._axis[i].get_min_max_acc(t1, t2)

            #distance from zero thrust point in this axis
            v1 = amin - self._grav[i] #left
            v2 = amax - self._grav[i] #right

            #definitely infeasible:
            if (max(v1**2, v2**2) > fmaxAllowed**2):
                return InputFeasibilityResult.InfeasibleThrustHigh

            if(v1*v2 < 0):
                #sign of acceleration changes, so we've gone through zero
                fminSqr += 0
            else:
                fminSqr += min(np.fabs(v1), np.fabs(v2))**2

            fmaxSqr += max(np.fabs(v1), np.fabs(v2))**2

            jmaxSqr += self._axis[i].get_max_jerk_squared(t1, t2)

        fmin = np.sqrt(fminSqr)
        fmax = np.sqrt(fmaxSqr)
        if fminSqr > 1e-6:
            wBound = np.sqrt(jmaxSqr / fminSqr)#the 1e-6 is a divide-by-zero protection
        else:
            wBound = float("inf")

        #definitely infeasible:
        if fmax < fminAllowed:
            return InputFeasibilityResult.InfeasibleThrustLow
        if fmin > fmaxAllowed:
            return InputFeasibilityResult.InfeasibleThrustHigh

        #possibly infeasible:
        if (fmin < fminAllowed) or (fmax > fmaxAllowed) or (wBound > wmaxAllowed):
            #indeterminate: must check more closely:
            tHalf = (t1 + t2) / 2.0
            r1 = self._check_input_feasibility_section(fminAllowed, fmaxAllowed, wmaxAllowed, minTimeSection, t1, tHalf)
            
            if r1 == InputFeasibilityResult.Feasible:
                #check the other half
                return self._check_input_feasibility_section(fminAllowed, fmaxAllowed, wmaxAllowed, minTimeSection, tHalf, t2)
            else:
                #infeasible, or indeterminable
                return r1

        #definitely feasible:
        return InputFeasibilityResult.Feasible

    def check_position_feasibility(self, boundaryPoint, boundaryNormal):
        """Test whether the position trajectory is allowable w.r.t. a plane.

        Test whether the position trajectory remains on the allowable side
        of a given plane. The plane is defined by giving a point on the plane,
        and the normal vector to the plane.
        
        The result is of the class StateFeasibilityResult, either Feasible or
        Infeasible.

        Args:
            boundaryPoint (array(3)): a point lying on the plane defining the 
                boundary.
            boundaryNormal (array(3)): a vector defining the normal of the 
                boundary. All points lying in the direction of the normal from
                the boundary are taken as feasible.

        Returns:
            An enumeration, of type StateFeasibilityResult.

        """

        boundaryNormal = np.array(boundaryNormal)
        boundaryPoint  = np.array(boundaryPoint)
        
        #make sure it's a unit vector:
        boundaryNormal = boundaryNormal/np.linalg.norm(boundaryNormal)

        #first, we will build the polynomial describing the velocity of the a
        #quadrocopter in the direction of the normal. Then we will solve for 
        #the zeros of this, which give us the times when the position is at a
        #critical point. Then we evaluate the position at these points, and at
        #the trajectory beginning and end, to see whether we are feasible. 
        
        coeffs = np.zeros(5)

        for i in range(3):
            coeffs[0] += boundaryNormal[i]*self._axis[i].get_param_alpha()/24.0            # t**4
            coeffs[1] += boundaryNormal[i]*self._axis[i].get_param_beta() /6.0             # t**3
            coeffs[2] += boundaryNormal[i]*self._axis[i].get_param_gamma()/2.0             # t**2
            coeffs[3] += boundaryNormal[i]*self._axis[i].get_initial_acceleration()        # t
            coeffs[4] += boundaryNormal[i]*self._axis[i].get_initial_velocity()            # 1
        
        #calculate the roots
        tRoots = np.roots(coeffs)
        
        #test these times, and the initial & end times:
        for t in np.append(tRoots,[0,self._tf]):
            distToPoint = np.dot(self.get_position(t) - boundaryPoint, boundaryNormal)
            if distToPoint <= 0:
                return StateFeasibilityResult.Infeasible
        
        #all points tested feasible:
        return StateFeasibilityResult.Feasible

    def get_jerk(self, t):
        """ Return the trajectory's 3D jerk value at time `t`."""
        return np.array([self._axis[i].get_jerk(t) for i in range(3)])

    def get_acceleration(self, t):
        """ Return the trajectory's 3D acceleration value at time `t`."""
        return np.array([self._axis[i].get_acceleration(t) for i in range(3)])

    def get_velocity(self, t):
        """ Return the trajectory's 3D velocity value at time `t`."""
        return np.array([self._axis[i].get_velocity(t) for i in range(3)])

    def get_position(self, t):
        ''' Return the trajectory's 3D position value at time `t`.'''
        return np.array([self._axis[i].get_position(t) for i in range(3)]).T
    
    def get_yaw(self, t):
        """ Return the trajectory's yaw orientation value at time `t`.
        Follow NED convention, 0 is the global X axis, in radians."""
        return np.arctan2(self._axis[1].get_velocity(t), self._axis[0].get_velocity(t))

    def get_normal_vector(self, t):
        """ Return the vehicle's normal vector at time `t`.

        The vehicle's normal vector is that vector along which the thrust
        points, e_3. The required body rates to fly a trajectory can be 
        calculated by finding that angular velocity which rotates this 
        normal vector from one direction to another. Note that the result
        will be expressed in the planning frame, so that a rotation is
        necessary to the body frame.

        Args:
            t (float): time argument.

        Returns:
            np.array() containing a unit vector.

        """
        v = (self.get_acceleration(t) - self._grav)
        return v/np.linalg.norm(v)

    def get_thrust(self, t):
        """ Return the thrust input at time `t`.

        Returns the thrust required at time `t` along the trajectory, in units
        of acceleration. 

        Args:
            t (float): time argument.

        Returns:
            np.array() containing a unit vector.

        """
        return np.linalg.norm(self.get_acceleration(t) - self._grav)

    def get_body_rates(self, t, dt=1e-3):
        """ Return the body rates input at time `t`, in inertial frame.

        Returns the body rates required at time `t` along the trajectory, in 
        units of [rad/s]. This is done by discretizing the normal direction
        trajectory, with discretization `dt`.
        
        **To get (p,q,r) rates, rotate these with the vehicle's attitude.**

        Args:
            t (float): time argument.
            dt (float, optional): discretization time, default is 1ms

        Returns:
            np.array() containing the rates, in the inertial frame.

        """
        n0 = self.get_normal_vector(t)
        n1 = self.get_normal_vector(t + dt)

        crossProd = np.cross(n0,n1) #direction of omega, in inertial axes

        if np.linalg.norm(crossProd) > 1e-6:
            return  np.arccos(np.dot(n0,n1))/dt*(crossProd/np.linalg.norm(crossProd))
        else:
            return np.array([0,0,0])


    def compute_cost(self, goal_point, goal_direction, edt):
        """ Return the total trajectory cost.

        Returns the total trajectory cost. Trajectories with higher cost will 
        tend to have more aggressive inputs (thrust and body rates), so that 
        this is a cheap way to compare two trajectories.
        """
        # Collision cost
        samplingCollision = np.int(np.clip(np.linalg.norm(self.get_position(self._tf) - self.get_position(0)) * 50, 10, 1e100))
        t = np.linspace(0, self._tf, samplingCollision)
        pos = self.get_position(t).astype(np.double)
        distances = np.vectorize(lambda x, y, z: edt([x, y, z]))(pos[:, 0], pos[:, 1], pos[:, 2])
        collisionCost = np.sum(1 / (distances + 1e-12)) * self._tf / samplingCollision

        # Low altitude cost
        altitudeCost = 0 if pos[-1, 2] > 1.5 else np.inf
        collisionCost += altitudeCost

        if collisionCost == np.inf:
            self._cost = collisionCost
            return self._cost

        # Local goal distance cost
        distCost = np.linalg.norm((goal_point - pos[-1]) * np.array([1, 1, 10]))

        # Local goal direction cost
        vf = self.get_velocity(self._tf)
        vf_unit = vf / np.linalg.norm(vf)
        directionCost = np.linalg.norm(vf_unit - goal_direction)

        # Final cost
        self._distance_cost = 5 * distCost
        self._collision_cost = 0 * collisionCost
        self._direction_cost = 5 * directionCost
        self._cost = self._distance_cost + self._collision_cost + self._direction_cost
        return self._cost
    
    def print_cost(self):
        return '{}\n|\tdistance: {}\n|\tcollision: {}\n|\tdirection: {}\n'\
            .format(self._cost, self._distance_cost, self._collision_cost, self._direction_cost)
    
    def get_param_alpha(self, axNum):
        """Return the three parameters alpha which defines the trajectory."""
        return self._axis[axNum].get_param_alpha()

    def get_param_beta(self, axNum):
        """Return the three parameters beta which defines the trajectory."""
        return self._axis[axNum].get_param_beta()

    def get_param_gamma(self, axNum):
        """Return the three parameters gamma which defines the trajectory."""
        return self._axis[axNum].get_param_gamma()

