#include "MotionPrimitiveLibrary.h"

MotionPrimitiveLibrary::MotionPrimitiveLibrary(const double tf) : _tf(tf) {
    _g = Vec3(0, 0, -9.80665);
}

void MotionPrimitiveLibrary::setInitState(Vec3 pos0, Vec3 vel0, Vec3 acc0) {
    _pos0 = Vec3(pos0);
    _vel0 = Vec3(vel0);
    _acc0 = Vec3(acc0);
}

void MotionPrimitiveLibrary::setLocalGoal(Vec3 goalPoint, Vec3 goalDir) {
    _goalPoint = Vec3(goalPoint);
    _goalDir = Vec3(goalDir);
}

MotionPrimitive* MotionPrimitiveLibrary::getTrajectory() {
    return _traj;
}

bool MotionPrimitiveLibrary::optimize() {
    double norm0 = _vel0.GetNorm2();
    double yaw0 = atan2(_vel0.y, _vel0.x);

}

MotionPrimitive MotionPrimitiveLibrary::buildTrajectory(const double normVelf, const double yawf, const double zf) {
    MotionPrimitive traj(_pos0, _vel0, _acc0, _g);
    traj.SetGoalPositionInAxis(2, zf);
    traj.SetGoalVelocity(Vec3(normVelf * cos(yawf), normVelf * sin(yawf), 0));
    traj.SetGoalAcceleration(Vec3(0, 0, 0));
    traj.Generate(_tf);
    return traj;
}