// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "cloud_segmentation/cloud_segmentation.h"

#include <glog/logging.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/png_io.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cloud_segmentation/viewer_utils.h"

CloudSegmentation::CloudSegmentation(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      config_(getConfigFromParam(nh_private)),
      tree_(new pcl::search::KdTree<pcl::PointXYZRGB>()),
      normals_(new pcl::PointCloud<pcl::Normal>),
      pcl_pointcloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
      segmented_cloud_(new pcl::PointCloud<PointXYZRGBLNormal>),
      colored_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
      visualize_(false) {
  pointcloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1,
                                  &CloudSegmentation::cloudCallback, this);

  segmented_cloud_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1000);

  // Region growing config.
  region_growing_.setSearchMethod(tree_);
  region_growing_.setMinClusterSize(config_.min_cluster_size);
  region_growing_.setMaxClusterSize(10000000);
  region_growing_.setNumberOfNeighbours(config_.number_of_neighbours);
  float smoothness_threshold_rad =
      config_.smoothness_threshold_deg / 180.0 * M_PI;
  region_growing_.setSmoothnessThreshold(smoothness_threshold_rad);
  region_growing_.setCurvatureThreshold(config_.curvature_threshold);

  nh_private_.param<bool>("viewer/enable", visualize_, visualize_);

  if (visualize_) {
    std::vector<float> camera_intrinsics_vec;
    nh_private.param<std::vector<float>>(
        "camera_intrinsics", camera_intrinsics_vec, camera_intrinsics_vec);

    Eigen::Matrix3f camera_intrinsics =
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
            camera_intrinsics_vec.data());

    vizualizer_thread_ = std::thread(&CloudSegmentation::visualize, this,
                                     camera_intrinsics, &colored_cloud_);

    nh_private_.param<bool>("viewer/write_frames_to_file",
                            write_frames_to_file_, write_frames_to_file_);
  }
}

CloudSegmentation::Config CloudSegmentation::getConfigFromParam(
    const ros::NodeHandle& nh_private) {
  Config config;

  nh_private.param("filtering/enable", config.enable_filtering,
                   config.enable_filtering);
  nh_private.param("filtering/leaf_size", config.leaf_size, config.leaf_size);
  nh_private.param("filtering/min_distance", config.min_distance,
                   config.min_distance);
  nh_private.param("filtering/max_distance", config.max_distance,
                   config.max_distance);

  nh_private.param("normal_estimation/max_depth_change_factor",
                   config.max_depth_change_factor,
                   config.max_depth_change_factor);
  nh_private.param("normal_estimation/normal_smoothing_size",
                   config.normal_smoothing_size, config.normal_smoothing_size);

  nh_private.param("region_growing/min_cluster_size", config.min_cluster_size,
                   config.min_cluster_size);
  nh_private.param("region_growing/number_of_neighbours",
                   config.number_of_neighbours, config.number_of_neighbours);
  nh_private.param("region_growing/smoothness_threshold_deg",
                   config.smoothness_threshold_deg,
                   config.smoothness_threshold_deg);
  nh_private.param("region_growing/curvature_threshold",
                   config.curvature_threshold, config.curvature_threshold);

  bool verbose_log = false;
  nh_private.param<bool>("debug/verbose_log", verbose_log, verbose_log);

  if (verbose_log) {
    FLAGS_stderrthreshold = 0;
  }

  return config;
}

void CloudSegmentation::visualize(
    const Eigen::Matrix3f& camera_intrinsics,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr* colored_cloud) {
  pcl::visualization::PCLVisualizer viewer("Cloud segmentation viewer");

  viewer.setBackgroundColor(0.15, 0.15, 0.15);

  pcl::visualization::Camera camera;
  Eigen::Matrix4f camera_extrinsics = Eigen::Matrix4f::Identity();
  computeCameraParams(camera_intrinsics, camera_extrinsics, &camera);
  viewer.setCameraParameters(camera);

  int frame = 1;
  while (!viewer.wasStopped()) {
    viewer.setCameraParameters(camera);

    if (updated_cloud_) {
      if (!viewer.updatePointCloud(*colored_cloud, "segmented_cloud")) {
        viewer.addPointCloud(*colored_cloud, "segmented_cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,
            "segmented_cloud");
      }
      if (write_frames_to_file_) {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << static_cast<unsigned>(frame);
        viewer.saveScreenshot("cloud_segmentation/frame_" + ss.str() + ".png");
      }

      ++frame;
      updated_cloud_ = false;
    }

    viewer.spinOnce(10);
  }
}

void CloudSegmentation::cloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  pcl::PCLPointCloud2::Ptr pointcloud2(new pcl::PCLPointCloud2());

  pcl_conversions::moveToPCL(*pointcloud_msg, *pointcloud2);

  pcl::StopWatch watch_all;

  pcl_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*pointcloud2, *pcl_pointcloud_);

  if (config_.enable_filtering) {
    // Downsample the input pointcloud.
    pcl_pointcloud_normals_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    filterPointcloud();
  }

  // Estimate pointcloud normals.
  tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdTreeNormalEstimation();

  // Region growing-based clustering.
  tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGB>);
  regionGrowing();

  LOG(INFO) << "Cloud segmentation completed in " << std::fixed
            << watch_all.getTimeSeconds() << " seconds." << std::endl
            << std::endl;

  // Extract the clusters into a labeled pointcloud.
  getSegmentedCloud();

  if (segmented_cloud_->size()) {
    pcl::toPCLPointCloud2(*segmented_cloud_, *pointcloud2);
  }

  sensor_msgs::PointCloud2 segmented_cloud_msg;
  pcl_conversions::moveFromPCL(*pointcloud2, segmented_cloud_msg);

  segmented_cloud_msg.header.stamp = pointcloud_msg->header.stamp;
  segmented_cloud_msg.header.frame_id = pointcloud_msg->header.frame_id;
  segmented_cloud_pub_.publish(segmented_cloud_msg);

  if (visualize_) {
    colored_cloud_ = getColoredCloud();
    updated_cloud_ = true;
  }
}

void CloudSegmentation::filterPointcloud() {
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud(pcl_pointcloud_);
  voxel_grid.setLeafSize(config_.leaf_size, config_.leaf_size,
                         config_.leaf_size);

  voxel_grid.setFilterFieldName("z");
  voxel_grid.setFilterLimits(config_.min_distance, config_.max_distance);

  voxel_grid.filter(*pcl_pointcloud_);
}

void CloudSegmentation::kdTreeNormalEstimation() {
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

  normal_estimator.setSearchMethod(tree_);
  normal_estimator.setInputCloud(pcl_pointcloud_);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals_);
}

void CloudSegmentation::regionGrowing() {
  region_growing_.setSearchMethod(tree_);
  region_growing_.setInputCloud(pcl_pointcloud_);
  region_growing_.setInputNormals(normals_);

  region_growing_.extract(clusters_);
}

void CloudSegmentation::getSegmentedCloud() {
  segmented_cloud_->clear();

  if (!clusters_.empty()) {
    segmented_cloud_->is_dense = pcl_pointcloud_->is_dense;

    for (size_t segment_idx = 0u; segment_idx < clusters_.size();
         ++segment_idx) {
      for (const auto& index : clusters_[segment_idx].indices) {
        PointXYZRGBLNormal point;

        point.x = *(pcl_pointcloud_->points[index].data);
        point.y = *(pcl_pointcloud_->points[index].data + 1);
        point.z = *(pcl_pointcloud_->points[index].data + 2);
        point.normal_x = normals_->points[index].normal_x;
        point.normal_y = normals_->points[index].normal_y;
        point.normal_z = normals_->points[index].normal_z;
        point.r = pcl_pointcloud_->points[index].r;
        point.g = pcl_pointcloud_->points[index].g;
        point.b = pcl_pointcloud_->points[index].b;
        point.label = segment_idx;

        segmented_cloud_->points.push_back(point);
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudSegmentation::getColoredCloud() {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty()) {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

    srand(static_cast<unsigned int>(time(nullptr)));
    std::vector<unsigned char> colors;
    for (std::size_t i_segment = 0; i_segment < clusters_.size(); i_segment++) {
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
    }

    colored_cloud->width = pcl_pointcloud_->width;
    colored_cloud->height = pcl_pointcloud_->height;
    colored_cloud->is_dense = pcl_pointcloud_->is_dense;

    int next_color = 0;
    for (const auto& i_segment : clusters_) {
      for (const auto& index : i_segment.indices) {
        pcl::PointXYZRGB point;
        point.x = *(pcl_pointcloud_->points[index].data);
        point.y = *(pcl_pointcloud_->points[index].data + 1);
        point.z = *(pcl_pointcloud_->points[index].data + 2);
        point.r = colors[3 * next_color];
        point.g = colors[3 * next_color + 1];
        point.b = colors[3 * next_color + 2];
        colored_cloud->points.push_back(point);
      }
      next_color++;
    }
  }

  return (colored_cloud);
}
