#include "ros/ros.h"
#include "std_msgs/String.h"
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sstream>


class LocalPlanner {
private:
    ros::NodeHandle nh;
    bool generateEDT;
    octomap::OcTree* tree;

public:
    void run(int argc, char **argv) {
        ros::init(argc, argv, "local_planner");
        if (!nh.getParam("/local_planner/generate_edt", generateEDT)) {
            generateEDT = true;
        }

        ros::Subscriber sub = nh.subscribe("/octomap_binary", 10, &LocalPlanner::octomap_cb, this);

        ros::Rate loop_rate(20);

        while (ros::ok()) {
            ROS_INFO("Hello");

            ros::spinOnce();
            loop_rate.sleep();
        }
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
};

int main(int argc, char **argv) {
    LocalPlanner node;
    node.run(argc, argv);
    return 0;
}