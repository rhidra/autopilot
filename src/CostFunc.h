/**
 * Cost functions used with the Brent method to optimize the motion primitive trajectory.
 * Each cost function is represented as a functor object.
 */

#include "MotionPrimitiveLibrary.h"

// Final yaw cost function
class yawCostFunc {
public:
    yawCostFunc(MotionPrimitiveLibrary* _mpl, double _norm, double _z) 
    : mpl(_mpl), norm(_norm), z(_z) {}

    double operator()(double yaw) const {
        return mpl->buildTrajectory(norm, yaw, z)
            .GetCost(mpl->getGoalPoint(), mpl->getGoalDir(), mpl->getEDT());
    }
    
private:
    MotionPrimitiveLibrary* mpl;
    double norm, z;
};

// Final velocity norm cost function
class normCostFunc {
public:
    normCostFunc(MotionPrimitiveLibrary* _mpl, double _yaw, double _z) 
    : mpl(_mpl), yaw(_yaw), z(_z) {}

    double operator()(double norm) const {
        return mpl->buildTrajectory(norm, yaw, z)
            .GetCost(mpl->getGoalPoint(), mpl->getGoalDir(), mpl->getEDT());
    }
    
private:
    MotionPrimitiveLibrary* mpl;
    double yaw, z;
};

// Final Z position cost function
class zCostFunc {
public:
    zCostFunc(MotionPrimitiveLibrary* _mpl, double _yaw, double _norm) 
    : mpl(_mpl), yaw(_yaw), norm(_norm) {}

    double operator()(double z) const {
        return mpl->buildTrajectory(norm, yaw, z)
            .GetCost(mpl->getGoalPoint(), mpl->getGoalDir(), mpl->getEDT());
    }
    
private:
    MotionPrimitiveLibrary* mpl;
    double yaw, norm;
};