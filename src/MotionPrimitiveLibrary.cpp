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

bool MotionPrimitiveLibrary::optimize() {
    using boost::math::tools::brent_find_minima;

    const double norm0 = _vel0.GetNorm2();
    const double yaw0 = atan2(_vel0.y, _vel0.x);

    std::cout << "*******************************" << std::endl;
    std::cout << "Optimizing..." << std::endl;
    std::cout << "\tInitial position " << _pos0 << std::endl;
    std::cout << "\tInitial velocity " << _vel0 << std::endl;
    std::cout << "\tInitial acceleration " << _acc0 << std::endl;
    std::cout << "\tInitial norm " << norm0 << std::endl;
    std::cout << "\tInitial yaw " << yaw0 << std::endl;

    // Optimize using the Brent's method, implemented in Boost
    // Optimize the yaw
    const yawCostFunc f1(_pos0, _vel0, _acc0, _goalPoint, _goalDir, norm0, _pos0.z, _tf, _edt, _trajs);
    std::pair<double, double> r = brent_find_minima(f1, yaw0 - PI * .5, yaw0 + PI * .5, 10);
    const double yaw = r.first;

    std::cout << "Yaw optimization result: " << r.first << std::endl;

    // Optimize the norm
    const normCostFunc f2(_pos0, _vel0, _acc0, _goalPoint, _goalDir, yaw, _pos0.z, _tf, _edt, _trajs);
    r = brent_find_minima(f2, std::max(norm0 - 4/_tf, .1), norm0 + .5/_tf, 10);
    const double norm = r.first;
    
    std::cout << "Norm optimization result: " << r.first << std::endl;
    
    // Optimize the z
    const double z = _pos0.z;

    std::cout << "Building the trajectory" << std::endl;
    _traj = buildTrajectory(_pos0, _vel0, _acc0, norm, yaw, z, _tf);
    if (_traj.GetCost(_goalPoint, _goalDir, _edt) > FEASIBLE_TRAJ_THRESHOLD) {
        return false;
    }
    return true;
}

MotionPrimitive MotionPrimitiveLibrary::buildTrajectory(
    const Vec3 pos0, const Vec3 vel0, const Vec3 acc0,
    const double normVelf, const double yawf, const double zf, const double tf
) {
    MotionPrimitive traj(pos0, vel0, acc0, Vec3(0, 0, -9.80665));
    traj.SetGoalPositionInAxis(2, zf);
    traj.SetGoalVelocity(Vec3(normVelf * cos(yawf), normVelf * sin(yawf), 0));
    traj.SetGoalAcceleration(Vec3(0, 0, 0));
    traj.Generate(tf);
    return traj;
}