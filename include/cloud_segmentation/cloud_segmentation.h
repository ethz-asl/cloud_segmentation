// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef CLOUD_SEGMENTATION_CLOUD_SEGMENTATION_H_
#define CLOUD_SEGMENTATION_CLOUD_SEGMENTATION_H_

#include <thread>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

struct PointXYZRGBLNormal {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  uint32_t label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBLNormal,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z,
                                   normal_z)(float, rgb, rgb)(uint32_t, label,
                                                              label))

class CloudSegmentation {
 public:
  struct Config {
    // Filtering.
    bool enable_filtering = false;
    float leaf_size = 0.02f;
    float min_distance = 0.0f;
    float max_distance = 3.0f;

    // Normal estimation.
    bool use_integral_image = false;
    float max_depth_change_factor = 0.01f;
    float normal_smoothing_size = 5.0f;

    // Region growing.
    int min_cluster_size = 30;
    int number_of_neighbours = 20;
    float smoothness_threshold_deg = 30.0f;
    float curvature_threshold = 0.5f;
  };

  CloudSegmentation(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  Config getConfigFromParam(const ros::NodeHandle& nh_private);

  void cloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void visualize(const Eigen::Matrix3f& camera_intrinsics,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr* colored_cloud);

 private:
  void filterPointcloud();

  void normalEstimation();

  // Normal estimation based on integral images. This method is faster, however
  // it can only be used when the cloud has not been filtered and hence is still
  // organized.
  void integralImageNormalEstimation();

  void kdTreeNormalEstimation();

  void regionGrowing();

  void getSegmentedCloud();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Config config_;

  // Subscriber.
  ros::Subscriber pointcloud_sub_;

  // Publishers.
  ros::Publisher segmented_cloud_pub_;

  // KDTree.
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_;

  // Normals.
  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  // Region growing.
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> region_growing_;

  // Visualization.
  bool visualize_;
  bool write_frames_to_file_;

  std::thread vizualizer_thread_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pointcloud_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_pointcloud_normals_;
  pcl::PointCloud<PointXYZRGBLNormal>::Ptr segmented_cloud_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_;
  bool updated_cloud_;

  std::vector<pcl::PointIndices> clusters_;
};

#endif  // CLOUD_SEGMENTATION_CLOUD_SEGMENTATION_H_
