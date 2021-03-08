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

import numpy as np


class SingleAxisTrajectory:
    """A trajectory along one axis.
    
    This is used to construct the optimal trajectory in one axis, planning
    in the jerk to achieve position, velocity, and/or acceleration final
    conditions. The trajectory is initialised with a position, velocity and
    acceleration. 
    
    The trajectory is optimal with respect to the integral of jerk squared.
    
    Do not use this in isolation, this useful through the "RapidTrajectory"
    class, which wraps three of these and allows to test input/state 
    feasibility.

    """

    def __init__(self, pos0, vel0, acc0):
        """Initialise the trajectory with starting state."""
        self._p0 = pos0
        self._v0 = vel0
        self._a0 = acc0
        self._pf = 0
        self._vf = 0
        self._af = 0
        self.reset()

    def set_goal_position(self, posf):
        """Define the goal position for a trajectory."""
        self._posGoalDefined = True
        self._pf = posf

    def set_goal_velocity(self, velf):
        """Define the goal velocity for a trajectory."""
        self._velGoalDefined = True
        self._vf = velf

    def set_goal_acceleration(self, accf):
        """Define the goal acceleration for a trajectory."""
        self._accGoalDefined = True
        self._af = accf

    def generate(self, Tf):
        """ Generate a trajectory of duration Tf.

        Generate a trajectory, using the previously defined goal end states 
        (such as position, velocity, and/or acceleration).

        """
        #define starting position:
        delta_a = self._af - self._a0
        delta_v = self._vf - self._v0 - self._a0*Tf
        delta_p = self._pf - self._p0 - self._v0*Tf - 0.5*self._a0*Tf*Tf

        #powers of the end time:
        T2 = Tf*Tf
        T3 = T2*Tf
        T4 = T3*Tf
        T5 = T4*Tf

        #solve the trajectories, depending on what's constrained:
        if self._posGoalDefined and self._velGoalDefined and self._accGoalDefined:
            self._a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5
            self._b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5
            self._g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5
        elif self._posGoalDefined and self._velGoalDefined:
            self._a = (-120*Tf*delta_v + 320*   delta_p)/T5
            self._b = (  72*T2*delta_v - 200*Tf*delta_p)/T5
            self._g = ( -12*T3*delta_v +  40*T2*delta_p)/T5
        elif self._posGoalDefined and self._accGoalDefined:
            self._a = (-15*T2*delta_a + 90*   delta_p)/(2*T5)
            self._b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5)
            self._g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5)
        elif self._velGoalDefined and self._accGoalDefined:
            self._a = 0
            self._b = ( 6*Tf*delta_a - 12*   delta_v)/T3
            self._g = (-2*T2*delta_a +  6*Tf*delta_v)/T3
        elif self._posGoalDefined:
            self._a =  20*delta_p/T5
            self._b = -20*delta_p/T4
            self._g =  10*delta_p/T3
        elif self._velGoalDefined:
            self._a = 0
            self._b =-3*delta_v/T3
            self._g = 3*delta_v/T2
        elif self._accGoalDefined:
            self._a = 0
            self._b = 0
            self._g = delta_a/Tf
        else:
            #Nothing to do!
            self._a = self._b = self._g = 0

        #Calculate the cost:
        self._cost =  (self._g**2) + self._b*self._g*Tf + (self._b**2)*T2/3.0 + self._a*self._g*T2/3.0 + self._a*self._b*T3/4.0 + (self._a**2)*T4/20.0
                
    def reset(self):
        """Reset the trajectory parameters."""
        self._cost = float("inf")
        self._accGoalDefined = self._velGoalDefined = self._posGoalDefined = False
        self._accPeakTimes = [None,None]
        pass
    
    def get_jerk(self, t):
        """Return the scalar jerk at time t."""
        return self._g  + self._b*t  + (1.0/2.0)*self._a*t*t
    
    def get_acceleration(self, t):
        """Return the scalar acceleration at time t."""
        return self._a0 + self._g*t  + (1.0/2.0)*self._b*t*t  + (1.0/6.0)*self._a*t*t*t

    def get_velocity(self, t):
        """Return the scalar velocity at time t."""
        return self._v0 + self._a0*t + (1.0/2.0)*self._g*t*t  + (1.0/6.0)*self._b*t*t*t + (1.0/24.0)*self._a*t*t*t*t

    def get_position(self, t):
        """Return the scalar position at time t."""
        return self._p0 + self._v0*t + (1.0/2.0)*self._a0*t*t + (1.0/6.0)*self._g*t*t*t + (1.0/24.0)*self._b*t*t*t*t + (1.0/120.0)*self._a*t*t*t*t*t

    def get_min_max_acc(self, t1, t2):
        """Return the extrema of the acceleration trajectory between t1 and t2."""
        if self._accPeakTimes[0] is None:
            #uninitialised: calculate the roots of the polynomial
            if self._a:
                #solve a quadratic
                det = self._b*self._b - 2*self._g*self._a
                if det<0:
                    #no real roots
                    self._accPeakTimes[0] = 0
                    self._accPeakTimes[1] = 0
                else:
                    self._accPeakTimes[0] = (-self._b + np.sqrt(det))/self._a
                    self._accPeakTimes[1] = (-self._b - np.sqrt(det))/self._a
            else:
                #_g + _b*t == 0:
                if self._b:
                    self._accPeakTimes[0] = -self._g/self._b
                    self._accPeakTimes[1] = 0
                else:
                    self._accPeakTimes[0] = 0
                    self._accPeakTimes[1] = 0

        #Evaluate the acceleration at the boundaries of the period:
        aMinOut = min(self.get_acceleration(t1), self.get_acceleration(t2))
        aMaxOut = max(self.get_acceleration(t1), self.get_acceleration(t2))

        #Evaluate at the maximum/minimum times:
        for i in [0,1]:
            if self._accPeakTimes[i] <= t1: continue
            if self._accPeakTimes[i] >= t2: continue
            
            aMinOut = min(aMinOut, self.get_acceleration(self._accPeakTimes[i]))
            aMaxOut = max(aMaxOut, self.get_acceleration(self._accPeakTimes[i]))
        return (aMinOut, aMaxOut)
 
    def get_max_jerk_squared(self,t1, t2):
        """Return the extrema of the jerk squared trajectory between t1 and t2."""
        jMaxSqr = max(self.get_jerk(t1)**2,self.get_jerk(t2)**2)
        
        if self._a:
            tMax = -self._b/self._a
            if(tMax>t1 and tMax<t2):
                jMaxSqr = max(pow(self.get_jerk(tMax),2),jMaxSqr)

        return jMaxSqr


    def get_param_alpha(self):
        """Return the parameter alpha which defines the trajectory."""
        return self._a

    def get_param_beta (self):
        """Return the parameter beta which defines the trajectory."""
        return self._b

    def get_param_gamma(self):
        """Return the parameter gamma which defines the trajectory."""
        return self._g

    def get_initial_acceleration(self):
        """Return the start acceleration of the trajectory."""
        return self._a0

    def get_initial_velocity(self):
        """Return the start velocity of the trajectory."""
        return self._v0

    def get_initial_position(self):
        """Return the start position of the trajectory."""
        return self._p0

    def get_cost(self):
        """Return the total cost of the trajectory."""
        return self._cost


#enums for feasibility results:
class InputFeasibilityResult:
    """An enumeration of the possible outcomes for the input feasiblity test.

    If the test does not return ``feasible``, it returns the outcome of the 
    first segment that fails. The different outcomes are:
        0: Feasible -- trajectory is feasible with respect to inputs
        1: Indeterminable -- a section's feasibility could not be determined
        2: InfeasibleThrustHigh -- a section failed due to max thrust constraint
        3: InfeasibleThrustLow -- a section failed due to min thrust constraint

    """
    Feasible, Indeterminable, InfeasibleThrustHigh, InfeasibleThrustLow = range(4)
    
    @classmethod
    def to_string(cls,ifr):
        """Return the name of the result."""
        if   ifr==InputFeasibilityResult.Feasible:
            return "Feasible"
        elif ifr==InputFeasibilityResult.Indeterminable:
            return "Indeterminable"
        elif  ifr==InputFeasibilityResult.InfeasibleThrustHigh:
            return "InfeasibleThrustHigh"
        elif ifr==InputFeasibilityResult.InfeasibleThrustLow:
            return "InfeasibleThrustLow"
        return "Unknown"

