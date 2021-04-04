#include "MotionPrimitiveLibrary.h"

MotionPrimitiveLibrary::MotionPrimitiveLibrary(const double tf) : _tf(tf) {}

void MotionPrimitiveLibrary::setInitState(Vec3 pos0, Vec3 vel0, Vec3 acc0) {
    _pos0 = Vec3(pos0);
    _vel0 = Vec3(vel0);
    _acc0 = Vec3(acc0);
}

void MotionPrimitiveLibrary::setLocalGoal(Vec3 goalPoint, Vec3 goalDir) {
    _goalPoint = Vec3(goalPoint);
    _goalDir = Vec3(goalDir);
}

void MotionPrimitiveLibrary::setEDT(DynamicEDTOctomap* edt) {
    _edt = edt;
}

MotionPrimitive* MotionPrimitiveLibrary::getTrajectory() {
    return _traj;
}
bool MotionPrimitiveLibrary::optimize() {
    using boost::math::tools::brent_find_minima;
    
    double norm0 = _vel0.GetNorm2();
    double yaw0 = atan2(_vel0.y, _vel0.x);

    // Optimize using the Brent's method, implemented in Boost
    // Optimize the yaw
    std::pair<double, double> r = brent_find_minima(yawCostFunc(this, norm0, _pos0.z), yaw0 - PI * .5, yaw0 + PI * .5, 3);
    double yaw = r.first;

    // Optimize the norm
    r = brent_find_minima(normCostFunc(this, yaw, _pos0.z), std::max(norm0 - 4/_tf, .1), norm0 + .5/_tf, 3);
    double norm = r.first;
    
    // Optimize the z
    double z = _pos0.z;

    _traj = &buildTrajectory(norm, yaw, z);
    if (_traj->GetCost(_goalPoint, _goalDir, _edt) > FEASIBLE_TRAJ_THRESHOLD) {
        return false;
    }
    return true;
}

MotionPrimitive MotionPrimitiveLibrary::buildTrajectory(const double normVelf, const double yawf, const double zf) {
    MotionPrimitive traj(_pos0, _vel0, _acc0, Vec3(0, 0, -9.80665));
    traj.SetGoalPositionInAxis(2, zf);
    traj.SetGoalVelocity(Vec3(normVelf * cos(yawf), normVelf * sin(yawf), 0));
    traj.SetGoalAcceleration(Vec3(0, 0, 0));
    traj.Generate(_tf);
    return traj;
}