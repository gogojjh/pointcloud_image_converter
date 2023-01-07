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
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/opencv.hpp>

struct LidarIntrinsics() {
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
}

class PointcloudToImage {
 public:
  PointcloudToImage(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

  void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void computeLidarIntrinsics(
      const pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud,
      pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud_filter,
      LidarIntrinsics &intr);

  void Pointcloud2DepthImage(
      const pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud_ptr,
      const LidarIntrinsics &intr, cv::Mat &depth_img);

  void Pointcloud2HeightImage(
      const pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud_ptr,
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

  // ROS message
  sensor_msgs::ImagePtr height_image_msg_ptr_;
  sensor_msgs::ImagePtr depth_image_msg_ptr_;
  sensor_msgs::CameraInfoPtr lidar_info_msg_ptr_;

  pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr input_cloud_ptr_;

  // Parameters
  LidarIntrinsics lidar_intrinsics_;

  float PC2IMG_SCALE_FACTOR;
  float PC2IMG_SCALE_OFFSET;

  int RAW_ELEVATION_DIVISIONS;
  int RAW_AZIMUTH_DIVISIONS;

  float VERTICAL_FOV_MIN;    // rad
  float VERTICAL_FOV_MAX;    // rad
  float HORIZONTAL_FOV_MIN;  // rad
  float HORIZONTAL_FOV_MAX;  // rad

  float SCAN_MIN_RANGE;
  float SCAN_MAX_RANGE;

  std::string DATASET_TYPE;
};

#endif  // POINTCLOUD_TO_IMAGE_H_
