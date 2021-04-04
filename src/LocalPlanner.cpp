#include "ros/ros.h"
#include "std_msgs/String.h"
#include "autopilot/LocalGoal.h"
#include "autopilot/MotionPrimitive.h"
#include "Vec3.h"
#include "MotionPrimitiveLibrary.h"
#include <controller_msgs/FlatTarget.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sstream>
#include <boost/numeric/ublas/vector.hpp>


class LocalPlanner {
private:
    ros::NodeHandle nh;
    bool generateEDT;
    octomap::OcTree* tree;
    ros::ServiceClient getLocalGoalSrv;
    ros::Publisher trajectory_pub;
    DynamicEDTOctomap* edt;
    double tf;

public:
    void run() {
        nh.param<bool>("/local_planner/generate_edt", generateEDT, true);
        nh.param<double>("/local_planner/tf", tf, 1);

        ros::Subscriber octomap_sub = nh.subscribe("/octomap_binary", 10, &LocalPlanner::octomap_cb, this);
        ros::Subscriber trajectory_sub = nh.subscribe("/autopilot/trajectory/request", 10, &LocalPlanner::sendTrajectory, this);
        trajectory_pub = nh.advertise<autopilot::MotionPrimitive>("/autopilot/trajectory/response", 10);
        getLocalGoalSrv = nh.serviceClient<autopilot::LocalGoal>("/autopilot/local_goal");

        getLocalGoalSrv.waitForExistence();
        ROS_INFO("Waiting for trajectory request...");

        ros::spin();
    }

    void octomap_cb(const octomap_msgs::Octomap& msg) { 
        tree = new octomap::OcTree(0.1);
        octomap::AbstractOcTree* temp = octomap_msgs::binaryMsgToMap(msg);
        tree = dynamic_cast<octomap::OcTree*>(temp);

        if (generateEDT) {
            ROS_INFO("Generating EDT...");
            double x,y,z;
            tree->getMetricMin(x,y,z);
            octomap::point3d min(x,y,z);
            tree->getMetricMax(x,y,z);
            octomap::point3d max(x,y,z);

            edt = new DynamicEDTOctomap(1.0, tree, min, max, false);
            edt->update(true);
            ROS_INFO("EDT generated");
        }
    }

    void sendTrajectory(const controller_msgs::FlatTarget& msg) {
        Vec3 pos0(msg.position.x, msg.position.y, msg.position.z);
        Vec3 vel0(msg.velocity.x, msg.velocity.y, msg.velocity.z);
        Vec3 acc0(0, 0, 0);

        autopilot::LocalGoal srv;
        srv.request.position = msg.position;
        srv.request.velocity = msg.velocity;
        
        if (!getLocalGoalSrv.call(srv)) {
            ROS_ERROR("Cannot get local goal !");
            exit(1);
        }
        Vec3 goalPoint(srv.response.local_goal_position.x, srv.response.local_goal_position.y, srv.response.local_goal_position.z);
        Vec3 goalDir(srv.response.local_goal_direction.x, srv.response.local_goal_direction.y, srv.response.local_goal_direction.z);

        MotionPrimitiveLibrary mpl(tf);
        mpl.setInitState(pos0, vel0, acc0);
        mpl.setLocalGoal(goalPoint, goalDir);
        mpl.setEDT(edt);
        if (!mpl.optimize()) {
            ROS_ERROR("Cannot optimize a feasible trajectory !");
            exit(1);
        }

        MotionPrimitive* traj = mpl.getTrajectory();
        trajectory_pub.publish(traj->toMsg());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    LocalPlanner node;
    node.run();
    return 0;
}