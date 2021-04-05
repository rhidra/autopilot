#ifndef MOTIONPRIMITIVELIBRARY_H_
#define MOTIONPRIMITIVELIBRARY_H_

#include "Vec3.h"
#include "MotionPrimitive.h"
#include "ros/ros.h"

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
    
    static MotionPrimitive buildTrajectory(
        const Vec3 pos0, const Vec3 vel0, const Vec3 acc0,
        const double normVelf, const double yawf, const double zf, const double tf
    );

    MotionPrimitive getTrajectory() { return _traj; };
    double getTf() { return _tf; };
    Vec3 getPos0() { return _pos0; };
    Vec3 getVel0() { return _vel0; };
    Vec3 getAcc0() { return _acc0; };
    Vec3 getGoalPoint() { return _goalPoint; };
    Vec3 getGoalDir() { return _goalDir; };
    DynamicEDTOctomap* getEDT() { return _edt; };
    std::vector<MotionPrimitive> getTrajs() { return _trajs; };

private:
    double _tf;
    Vec3 _pos0, _vel0, _acc0;
    Vec3 _goalPoint, _goalDir;
    DynamicEDTOctomap* _edt;
    std::vector<MotionPrimitive> _trajs;
    MotionPrimitive _traj;
};

//* Cost functions *//

// Final yaw cost function
class yawCostFunc {
public:
    yawCostFunc(Vec3 _pos0, Vec3 _vel0, Vec3 _acc0, Vec3 _goalPoint, Vec3 _goalDir, double _norm, double _z, 
                double _tf, DynamicEDTOctomap *_edt, std::vector<MotionPrimitive> &_trajs) 
    :   pos0(_pos0), vel0(_vel0), acc0(_acc0), goalPoint(_goalPoint), goalDir(_goalDir), 
        norm(_norm), z(_z), tf(_tf), edt(_edt), trajs(_trajs) {}

    double operator()(double yaw) {
        std::cout << "\tYaw: " << yaw << std::endl;
        MotionPrimitive t = MotionPrimitiveLibrary::buildTrajectory(pos0, vel0, acc0, norm, yaw, z, tf);
        trajs.push_back(t);
        return t.GetCost(goalPoint, goalDir, edt);
    }
    
private:
    Vec3 pos0, vel0, acc0;
    Vec3 goalPoint, goalDir;
    DynamicEDTOctomap* edt;
    double norm, z, tf;
    std::vector<MotionPrimitive> &trajs;
};

// Final velocity norm cost function
class normCostFunc {
public:
    normCostFunc(Vec3 _pos0, Vec3 _vel0, Vec3 _acc0, Vec3 _goalPoint, Vec3 _goalDir, double _yaw, double _z, 
                double _tf, DynamicEDTOctomap *_edt, std::vector<MotionPrimitive> &_trajs) 
    :   pos0(_pos0), vel0(_vel0), acc0(_acc0), goalPoint(_goalPoint), goalDir(_goalDir), 
        yaw(_yaw), z(_z), tf(_tf), edt(_edt), trajs(_trajs) {}

    double operator()(double norm) {
        std::cout << "\tNorm: " << yaw << std::endl;
        MotionPrimitive t = MotionPrimitiveLibrary::buildTrajectory(pos0, vel0, acc0, norm, yaw, z, tf);
        trajs.push_back(t);
        return t.GetCost(goalPoint, goalDir, edt);
    }
    
private:
    Vec3 pos0, vel0, acc0;
    Vec3 goalPoint, goalDir;
    DynamicEDTOctomap* edt;
    double yaw, z, tf;
    std::vector<MotionPrimitive> &trajs;
};

// Final Z position cost function
class zCostFunc {
public:
    zCostFunc(Vec3 _pos0, Vec3 _vel0, Vec3 _acc0, Vec3 _goalPoint, Vec3 _goalDir, double _norm, double _yaw, 
                double _tf, DynamicEDTOctomap *_edt, std::vector<MotionPrimitive> &_trajs) 
    :   pos0(_pos0), vel0(_vel0), acc0(_acc0), goalPoint(_goalPoint), goalDir(_goalDir), 
        norm(_norm), yaw(_yaw), tf(_tf), edt(_edt), trajs(_trajs) {}

    double operator()(double z) const {
        std::cout << "\tZ: " << yaw << std::endl;
        MotionPrimitive t = MotionPrimitiveLibrary::buildTrajectory(pos0, vel0, acc0, norm, yaw, z, tf);
        trajs.push_back(t);
        return t.GetCost(goalPoint, goalDir, edt);
    }
    
private:
    Vec3 pos0, vel0, acc0;
    Vec3 goalPoint, goalDir;
    DynamicEDTOctomap* edt;
    double norm, yaw, tf;
    std::vector<MotionPrimitive> &trajs;
};

#endif