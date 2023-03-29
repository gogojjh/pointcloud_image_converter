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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <deque>
#include <opencv2/opencv.hpp>
#include <thread>

#include "pointcloud_image_converter/ouster_point.hpp"
#include "pointcloud_image_converter/point_xyzirgbl.hpp"

namespace pc_img_conv {

typedef ouster_ros::Point PointType;

struct LidarIntrinsics {
  std::string distortion_model_;

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
    std::cout << distortion_model_ << " ";
    std::cout << num_azimuth_divisions_ << " " << num_elevation_divisions_
              << " " << horizontal_fov_ << " " << vertical_fov_ << " "
              << start_azimuth_rad_ << " " << end_azimuth_rad_ << " "
              << start_elevation_rad_ << " " << end_elevation_rad_ << std::endl;
  }

  void ConvertToMsg(sensor_msgs::CameraInfoPtr &lidar_info_msg_ptr) const {
    lidar_info_msg_ptr->height = num_elevation_divisions_;
    lidar_info_msg_ptr->width = num_azimuth_divisions_;
    lidar_info_msg_ptr->distortion_model = distortion_model_;
    lidar_info_msg_ptr->D = {static_cast<float>(num_azimuth_divisions_),
                             static_cast<float>(num_elevation_divisions_),
                             horizontal_fov_,
                             vertical_fov_,
                             start_azimuth_rad_,
                             end_azimuth_rad_,
                             start_elevation_rad_,
                             end_elevation_rad_};
  }
};

class PointCloudToImage {
 public:
  PointCloudToImage(ros::NodeHandle &nh);

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void ImageCallback(const sensor_msgs::Image::ConstPtr &msg);

  void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

  bool computeLidarIntrinsics(const pcl::PointCloud<PointType>::Ptr &cloud,
                              pcl::PointCloud<PointType>::Ptr &cloud_filter,
                              LidarIntrinsics &intr);

  void Pointcloud2DepthImage(const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
                             const LidarIntrinsics &intr, cv::Mat &img);

  void Pointcloud2HeightImage(const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
                              const LidarIntrinsics &intr, cv::Mat &img);

  void Pointcloud2SemanticImage(
      const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
      const LidarIntrinsics &intr, cv::Mat &img);

  void ProcessPointCloudImageAlignment(const sensor_msgs::ImageConstPtr &msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS Subscriber
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  tf::TransformListener listener_;

  // ROS Publisher
  ros::Publisher depth_pub_;
  ros::Publisher height_pub_;
  ros::Publisher semantic_pub_;
  ros::Publisher lidar_info_pub_;

  std::queue<std::tuple<std_msgs::Header, pcl::PointCloud<PointType>::Ptr,
                        LidarIntrinsics>>
      cloud_queue_;

  // Parameters
  LidarIntrinsics lidar_intrinsics_;
  Eigen::Matrix3d K_;

  float PC2IMG_SCALE_FACTOR;
  float PC2IMG_SCALE_OFFSET;

  float SCAN_MIN_RANGE;
  float SCAN_MAX_RANGE;

  std::string DATASET_TYPE;
  float MIN_T_DATA;

  int CLOUD_BUFF;

  // Other
  std::mutex mutex_cloud_;
};
}  // namespace pc_img_conv

#endif  // POINTCLOUD_TO_IMAGE_H_
