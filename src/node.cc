#include <glog/logging.h>
#include <ros/ros.h>

#include "cloud_segmentation/cloud_segmentation.h"

int main(int argc, char** argv) {
  std::cout << std::endl
            << "Point cloud segmentation ROS node - Copyright (c) 2020- "
               "Margarita Grinvald, Autonomous "
               "Systems Lab, ETH Zurich."
            << std::endl
            << std::endl;

  ros::init(argc, argv, "cloud_segmentation_node");
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  CloudSegmentation cloud_segmentation(nh, nh_private);

  ros::spin();

  return 0;
}
