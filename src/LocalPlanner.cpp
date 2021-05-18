#include "ros/ros.h"
#include "autopilot/LocalGoal.h"
#include "autopilot/MotionPrimitive.h"
#include "autopilot/BoundingBox.h"
#include "Vec3.h"
#include "MotionPrimitiveLibrary.h"
#include <controller_msgs/FlatTarget.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sstream>
#include <boost/numeric/ublas/vector.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


class LocalPlanner {
private:
    ros::NodeHandle nh;
    bool generateEDT, updateOctomap;
    octomap::OcTree* tree;
    ros::ServiceClient getLocalGoalSrv;
    ros::Publisher trajectory_pub;
    ros::Publisher viz_pub;
    DynamicEDTOctomap* edt;
    double tf;
    double xmin, xmax, ymin, ymax, zmin, zmax;
    int i;

public:
    void run() {
        i = 0;
        nh.param<bool>("/local_planner/generate_edt", generateEDT, true);
        nh.param<bool>("/local_planner/update_octomap", updateOctomap, true);
        nh.param<double>("/local_planner/tf", tf, 1);
        nh.param<double>("/world/x/min", xmin, -20);
        nh.param<double>("/world/x/max", xmax, 20);
        nh.param<double>("/world/y/min", ymin, -20);
        nh.param<double>("/world/y/max", ymax, 20);
        nh.param<double>("/world/z/min", zmin, 0);
        nh.param<double>("/world/z/max", zmax, 4);

        ros::Subscriber octomap_sub = nh.subscribe("/octomap_binary", 10, &LocalPlanner::octomap_cb, this);
        ros::Subscriber octomap_update_sub = nh.subscribe("/autopilot/octomap_update", 10, &LocalPlanner::octomap_update_cb, this);
        ros::Subscriber trajectory_sub = nh.subscribe("/autopilot/trajectory/request", 10, &LocalPlanner::sendTrajectory, this);
        trajectory_pub = nh.advertise<autopilot::MotionPrimitive>("/autopilot/trajectory/response", 10);
        viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/autopilot/viz/local", 10);
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
            // double x,y,z;
            // tree->getMetricMin(x,y,z);
            octomap::point3d min(xmin, ymin, zmin);
            // tree->getMetricMax(x,y,z);
            octomap::point3d max(xmax, ymax, zmax);

            edt = new DynamicEDTOctomap(1.0, tree, min, max, false);
            edt->update(true);
            ROS_INFO("EDT generated");
        }
    }

    void octomap_update_cb(const autopilot::BoundingBox& msg) {
        if (updateOctomap) {
            ROS_INFO("Updating the octomap");
            for (int i = 0; i < msg.n; i++) {
                double c2 = i / ((double) msg.n);
                double c1 = 1. - c2;
                double x =  c1 * msg.min.x + c2 * msg.max.x;
                double y =  c1 * msg.min.y + c2 * msg.max.y;
                double z =  c1 * msg.min.z + c2 * msg.max.z;
                tree->updateNode(x, y, z, true);
            }

            if (generateEDT) {
                ROS_INFO("Updating EDT...");
                edt->update(true);
            }
            ROS_INFO("Done updating the octomap");
        }
    }

    void sendTrajectory(const controller_msgs::FlatTarget& msg) {
        i++;
        ros::Time start = ros::Time::now();
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
            nh.setParam("/autopilot/done", 2);
            viz_pub.publish(vizMPL(mpl, i));
            exit(1);
        }

        MotionPrimitive traj = mpl.getTrajectory();
        float generationTime = (ros::Time::now() - start).toNSec()/1000000.;
        trajectory_pub.publish(traj.toMsg(generationTime));

        std::cout << "Trajectory generated in " << (ros::Time::now() - start).toNSec()/1000000. << " ms" << std::endl;

        viz_pub.publish(vizMPL(mpl, i, true));
    }

    visualization_msgs::MarkerArray vizMPL(MotionPrimitiveLibrary mpl, int id = 0, bool showSelectedTrajectory = false) {
        visualization_msgs::MarkerArray ma;
        int i = id*200;

        ma.markers.push_back(vizPoint(mpl.getPos0(), .05, 1, 1, 1, .7, i-1));

        if (showSelectedTrajectory) {
            ma.markers.push_back(vizTraj(mpl.getTrajectory(), 0, 1, 1, 1, i));
            ma.markers.push_back(vizPoint(mpl.getTrajectory().GetPosition(tf), .05, 0, 1, 1, .3, i));
        }

        for (const MotionPrimitive& traj: mpl.getTrajs()) {
            i++;
            double d = std::min(traj._cost / 100, 1.);
            ma.markers.push_back(vizTraj(traj, d, 1-d, 0, .2, i));
            ma.markers.push_back(vizPoint(traj.GetPosition(tf), .03, d, 1-d, 0, .2, i));
        }
        return ma;
    }

    visualization_msgs::Marker vizTraj(MotionPrimitive const traj, float r = 1, float g = 1, float b = 1, float alpha = 1, int id = 0) {
        visualization_msgs::Marker m;
        m.header.frame_id = "/map";
        m.header.stamp = ros::Time::now();
        m.ns = "path";
        m.action = visualization_msgs::Marker::ADD;
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.scale.x = .01;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = alpha;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;

        for (int i = 0; i < 10; ++i) {
            Vec3 pos1 = traj.GetPosition(i / 10. * tf);
            Vec3 pos2 = traj.GetPosition((i+1) / 10. * tf);
            geometry_msgs::Point pt1, pt2;
            pt1.x = pos1.x; pt1.y = pos1.y; pt1.z = pos1.z;
            pt2.x = pos2.x; pt2.y = pos2.y; pt2.z = pos2.z;
            m.points.push_back(pt1);
            m.points.push_back(pt2);
        }

        return m;
    }

    visualization_msgs::Marker vizPoint(Vec3 p, float size, float r, float g, float b, float alpha = 1, int id = 0) {
        visualization_msgs::Marker m;
        m.header.frame_id = "/map";
        m.header.stamp = ros::Time::now();
        m.ns = "point";
        m.action = visualization_msgs::Marker::ADD;
        m.id = id;
        m.type = visualization_msgs::Marker::SPHERE;
        m.scale.x = size;
        m.scale.y = size;
        m.scale.z = size;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = alpha;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = p.z;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        return m;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    LocalPlanner node;
    node.run();
    return 0;
}