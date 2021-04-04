#pragma once

#include "Vec3.h"
#include "MotionPrimitive.h"
#include "CostFunc.h"

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <vector>
#include <math.h>
#include <boost/math/tools/minima.hpp>

#define PI 3.14159265358979323846

#define FEASIBLE_TRAJ_THRESHOLD 1000

class MotionPrimitiveLibrary {
public:
    MotionPrimitiveLibrary(const double tf);
    void setInitState(const Vec3 pos0, const Vec3 vel0, const Vec3 acc0);
    void setLocalGoal(Vec3 goalPoint, Vec3 goalDir);
    void setEDT(DynamicEDTOctomap* edt);
    bool optimize();
    
    MotionPrimitive buildTrajectory(const double normVelf, const double yawf, const double zf);

    MotionPrimitive* getTrajectory();
    double getTf() { return _tf; };
    Vec3 getPos0() { return _pos0; };
    Vec3 getVel0() { return _vel0; };
    Vec3 getAcc0() { return _acc0; };
    Vec3 getGoalPoint() { return _goalPoint; };
    Vec3 getGoalDir() { return _goalDir; };
    DynamicEDTOctomap* getEDT() { return _edt; };

private:
    double _tf;
    Vec3 _pos0, _vel0, _acc0;
    Vec3 _goalPoint, _goalDir;
    DynamicEDTOctomap* _edt;
    std::vector<MotionPrimitive> _trajs;
    MotionPrimitive* _traj;
};