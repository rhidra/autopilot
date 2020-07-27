#include "ros/ros.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>


template<class TreeType>
void readTree(TreeType* octree, const octomap_msgs::Octomap::ConstPtr& msg){
  std::stringstream datastream;
  if (msg->data.size() > 0){
    datastream.write((const char*) &msg->data[0], msg->data.size());
    octree->readBinaryData(datastream);
  }
}

void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg) {

  octomap::OcTree* tree = new octomap::OcTree(msg->resolution);
  readTree(tree, msg);

  octomap::point3d query(0., 0., 0.);
  octomap::OcTreeNode* cell = tree->search(query);
  ROS_INFO("Occupancy1: %f", cell->getOccupancy());

  query = octomap::point3d(-1., -1., 0.);
  cell = tree->search(query);
  ROS_INFO("Occupancy2: %f", cell->getOccupancy());

  query = octomap::point3d(1.1, 1.07, 0.5);
  cell = tree->search(query);
  ROS_INFO("Occupancy3: %f", cell->getOccupancy());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Subscriber octomap_sub = n.subscribe<octomap_msgs::Octomap>("octomap_binary", 1000, octomap_cb);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
