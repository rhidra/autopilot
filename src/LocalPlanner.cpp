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

public:
    void run(int argc, char **argv) {
        ros::init(argc, argv, "local_planner");
        if (!nh.getParam("/local_planner/generate_edt", generateEDT)) {
            generateEDT = true;
        }

        ros::Subscriber octomap_sub = nh.subscribe("/octomap_binary", 10, &LocalPlanner::octomap_cb, this);
        ros::Subscriber trajectory_sub = nh.subscribe("/autopilot/trajectory/request", 10, &LocalPlanner::sendTrajectory, this);
        ros::Publisher trajectory_pub = nh.advertise<autopilot::MotionPrimitive>("/autopilot/trajectory/response", 10);
        getLocalGoalSrv = nh.serviceClient<autopilot::LocalGoal>("/autopilot/local_goal");

        getLocalGoalSrv.waitForExistence();
        ROS_INFO("Waiting for trajectory request...");

        ros::spin();
    }

    void octomap_cb(const octomap_msgs::Octomap& msg) { 
        // tree = new octomap::OcTree(0.1);
        octomap::AbstractOcTree* temp = octomap_msgs::fullMsgToMap(msg);
        tree = dynamic_cast<octomap::OcTree*>(temp);

        if (generateEDT) {
            ROS_INFO("Generating EDT...");
            double x,y,z;
            tree->getMetricMin(x,y,z);
            octomap::point3d min(x,y,z);
            tree->getMetricMax(x,y,z);
            octomap::point3d max(x,y,z);

            DynamicEDTOctomap distmap(50.0, tree, min, max, false);
            distmap.update(true);
        }
    }

    void sendTrajectory(const controller_msgs::FlatTarget& msg) {
        Vec3 pos0(msg.position.x, msg.position.y, msg.position.z);
        Vec3 vel0(msg.velocity.x, msg.velocity.y, msg.velocity.z);
        Vec3 acc0(0, 0, 0);
        MotionPrimitiveLibrary mpl();
    }
};

int main(int argc, char **argv) {
    LocalPlanner node;
    node.run(argc, argv);
    return 0;
}