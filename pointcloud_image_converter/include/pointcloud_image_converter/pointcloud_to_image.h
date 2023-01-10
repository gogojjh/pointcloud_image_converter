//
// Created by kevinlad on 23-1-2.
//

#ifndef POINTCLOUD_TO_IMAGE_H_
#define POINTCLOUD_TO_IMAGE_H_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include "pointcloud_image_converter/pcl/point_xyzirrat.hpp"

namespace pc_img_conv {

// NOTE(gogojjh): ouster lidar
typedef pcl::PointXYZIRRAT PointType;

// NOTE(gogojjh): common lidar
// typedef pcl::PointXYZI PointType;

struct LidarIntrinsics {
  int num_elevation_divisions_;
  int num_azimuth_divisions_;

  float start_elevation_rad_;
  float end_elevation_rad_;
  float vertical_fov_;

  float start_azimuth_rad_;
  float end_azimuth_rad_;
  float horizontal_fov_;

  float rads_per_pixel_elevation_;
  float rads_per_pixel_azimuth_;

  void PrintIntrinsics() const {
    std::cout << num_azimuth_divisions_ << " " << num_elevation_divisions_
              << " " << horizontal_fov_ << " " << vertical_fov_ << " "
              << start_azimuth_rad_ << " " << end_azimuth_rad_ << " "
              << start_elevation_rad_ << " " << end_elevation_rad_ << std::endl;
  }
};

class PointCloudToImage {
 public:
  PointCloudToImage(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  bool computeLidarIntrinsics(const pcl::PointCloud<PointType>::Ptr &cloud,
                              pcl::PointCloud<PointType>::Ptr &cloud_filter,
                              LidarIntrinsics &intr);

  void Pointcloud2DepthImage(const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
                             const LidarIntrinsics &intr, cv::Mat &depth_img);

  void Pointcloud2HeightImage(const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
                              const LidarIntrinsics &intr, cv::Mat &height_img);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS Subscriber
  ros::Subscriber pointcloud_sub_;

  // ROS Publisher
  image_transport::CameraPublisher depth_pub_;
  image_transport::CameraPublisher height_pub_;
  std::shared_ptr<image_transport::ImageTransport> it_ptr_;

  // Parameters
  LidarIntrinsics lidar_intrinsics_;

  float PC2IMG_SCALE_FACTOR;
  float PC2IMG_SCALE_OFFSET;

  float SCAN_MIN_RANGE;
  float SCAN_MAX_RANGE;

  std::string DATASET_TYPE;
};
}  // namespace pc_img_conv

#endif  // POINTCLOUD_TO_IMAGE_H_
