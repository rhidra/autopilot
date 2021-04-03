#pragma once

#include "Vec3.h"
#include "MotionPrimitive.h"

#include <vector>
#include <math.h>

class MotionPrimitiveLibrary {
public:
    MotionPrimitiveLibrary(const double tf);
    void setInitState(const Vec3 pos0, const Vec3 vel0, const Vec3 acc0);
    void setLocalGoal(Vec3 goalPoint, Vec3 goalDir);
    bool optimize();
    MotionPrimitive buildTrajectory(const double normVelf, const double yawf, const double zf);
    MotionPrimitive* getTrajectory();

private:
    double _tf;
    Vec3 _pos0, _vel0, _acc0;
    Vec3 _goalPoint, _goalDir;
    std::vector<MotionPrimitive> _trajs;
    MotionPrimitive* _traj;
    Vec3 _g;
};