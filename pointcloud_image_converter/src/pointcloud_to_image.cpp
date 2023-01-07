//
// Created by kevinlad on 23-1-2.
//

#include "pointcloud_image_converter/pointcloud_to_image.h"

#include "pointcloud_image_converter/pcl/point_xyzirrat.hpp"

PointcloudToImage::PointcloudToImage(ros::NodeHandle &nh,
                                     ros::NodeHandle &nh_private)
    : nh(nh_), nh_private_(nh_private) {
  pointcloud_sub_ = nh_.subscribe("input_cloud", 1,
                                  &PointcloudToImage::PointcloudCallback, this);
  input_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZIRRAT>);
  lidar_info_msg_ptr_.reset(new sensor_msgs::CameraInfo);

  it_ptr_.reset(new image_transport::ImageTransport(nh_));
  depth_pub_ = it_ptr_->advertiseCamera("depth_image", 1);
  height_pub_ = it_ptr_->advertiseCamera("height_image", 1);

  // Parameters
  // assume that all points are higher than -10.0m
  // points at height within [-10m, 55m] can be stored
}

void PointCloudToImage::computeLidarIntrinsics(
    const pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud,
    pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud_filter,
    LidarIntrinsics &intr) {
  cloud_filter->reserve(cloud->size());

  // NOTE(gogojjh): depend on the specific dataset
  if (!DATASET_TYPE.find("FusionPortable") != std::string::npos) {
    intr.start_elevation_rad_ = 0.0f;
    intr.end_elevation_rad_ = 0.0f;
    int min_ring = 1000, max_ring = -1000;
    for (const auto &pt : *input_cloud_ptr_) {
      max_ring = std::max(max_ring, int(pt.ring));
      min_ring = std::min(min_ring, int(pt.ring));
    }
    int num_rings = max_ring - min_ring + 1;
    ROS_INFO("min_ring: %d  max_ring: %d", min_ring, max_ring);

    // NOTE(gogojjh): if all points are removed, output the default values
    if (num_rings > intr.num_elevation_divisions_) {
      intr.start_elevation_rad_ = M_PI / 2 - intr.vertical_fov_ / 2;
      intr.end_elevation_rad_ = M_PI / 2 + intr.vertical_fov_ / 2;
      intr.rads_per_pixel_elevation_ =
          (intr.end_elevation_rad_ - intr.start_elevation_rad_) /
          (intr.num_elevation_divisions_ - 1);
      intr.rads_per_pixel_azimuth_ =
          intr.horizontal_fov_ / (intr.num_azimuth_divisions_ - 1);
    } else {
      int cnt_start = 0;
      int cnt_end = 0;
      for (const auto &pt : *input_cloud_ptr_) {
        float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        // NOTE(gogojjh): Filter out points that are too close
        // Otherwise it will produce some NaN values
        if (r < SCAN_MIN_RANGE || r > SCAN_MAX_RANGE) continue;
        cloud_filter->push_back(pt);

        float elevation_angle_rad = acos(pt.z / r);
        if (int(pt.ring) == min_ring) {
          intr.start_elevation_rad_ += elevation_angle_rad;
          cnt_start++;
        }
        if (int(pt.ring) == max_ring) {
          intr.end_elevation_rad_ += elevation_angle_rad;
          cnt_end++;
        }
      }
      intr.start_elevation_rad_ /= cnt_start;
      intr.end_elevation_rad_ /= cnt_end;
      std::cout << "start_elevation_rad: " << intr.start_elevation_rad_
                << std::endl;
      std::cout << "end_elevation_rad: " << intr.end_elevation_rad_
                << std::endl;

      intr.rads_per_pixel_elevation_ =
          (intr.end_elevation_rad_ - intr.start_elevation_rad_) /
          (num_rings - 1);
      intr.rads_per_pixel_azimuth_ =
          intr.horizontal_fov_ / (intr.num_azimuth_divisions_ - 1);
      intr.start_elevation_rad_ -= intr.rads_per_pixel_elevation_ * min_ring;
      intr.end_elevation_rad_ += intr.rads_per_pixel_elevation_ *
                                 (intr.num_elevation_divisions_ - 1 - max_ring);
      intr.vertical_fov_ = intr.end_elevation_rad_ - intr.start_elevation_rad_;
    }
  } else if (!DATASET_TYPE.find("KITTI") != std::string::npos) {
  } else if (!DATASET_TYPE.find("SemanticKITTI") != std::string::npos) {
  } else if (!DATASET_TYPE.find("SemanticUSL") != std::string::npos) {
  }

  std::cout << intr.num_azimuth_divisions_ << " "
            << intr.num_elevation_divisions_ << " " << intr.horizontal_fov_
            << " " << intr.vertical_fov_ << " " << intr.start_azimuth_rad_
            << " " << intr.end_azimuth_rad_ << " " << intr.start_elevation_rad_
            << " " << intr.end_elevation_rad_ << std::endl;
}

void PointcloudToImage::Pointcloud2DepthImage(
    const pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &depth_img) {
  // NOTE(gogojjh): convert range to into millimeter.
  for (const auto &pt : *cloud_ptr) {
    float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    float elevation_angle_rad = acos(pt.z / r);
    float azimuth_angle_rad = M_PI - atan2(pt.y, pt.x);

    int row_id = std::round((elevation_angle_rad - start_elevation_rad) /
                            intr.rads_per_pixel_elevation);
    if (row_id < 0 || row_id > intr.num_elevation_divisions - 1) continue;

    int col_id = std::round(azimuth_angle_rad / intr.rads_per_pixel_azimuth);
    if (col_id >= num_azimuth_divisions) col_id -= intr.num_azimuth_divisions;

    float dep = r * PC2IMG_SCALE_FACTOR;
    if (dep > std::numeric_limits<uint16_t>::max()) {
      ROS_WARN("depth value is too large: %f", dep);
      continue;
    }
    if (dep < 0.0f) continue;
    depth_img.at<uint16_t>(row_id, col_id) = uint16_t(dep);
  }
}

void PointcloudToImage::Pointcloud2HeightImage(
    const pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &height_img) {
  // NOTE(gogojjh): convert range to into millimeter.
  for (const auto &pt : *input_cloud_ptr_) {
    float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    float elevation_angle_rad = acos(pt.z / r);
    float azimuth_angle_rad = M_PI - atan2(pt.y, pt.x);

    int row_id = round((elevation_angle_rad - start_elevation_rad) /
                       rads_per_pixel_elevation);
    if (row_id < 0 || row_id > num_elevation_divisions - 1) continue;

    int col_id = round(azimuth_angle_rad / rads_per_pixel_azimuth);
    if (col_id >= num_azimuth_divisions) col_id -= num_azimuth_divisions;

    float z = (pt.z + PC2IMG_SCALE_OFFSET) * PC2IMG_SCALE_FACTOR;
    if (z > std::numeric_limits<uint16_t>::max()) continue;
    if (z < 0.0f) continue;
    height_image.at<uint16_t>(row_id, col_id) = uint16_t(z);
  }
}

void PointcloudToImage::PointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *input_cloud_ptr_);
  auto frame_id = msg->header.frame_id;
  auto stamp = msg->header.stamp;
  ROS_INFO("Pointcloud received: got %zu points", input_cloud_ptr_->size());

  // NOTE(gogojjh): Set default lidar intrinsics
  lidar_intrinsics_.num_elevation_divisions_ = RAW_ELEVATION_DIVISIONS;
  lidar_intrinsics_.num_azimuth_divisions_ = RAW_AZIMUTH_DIVISIONS;

  lidar_intrinsics_.start_elevation_rad_ = VERTICAL_FOV_MIN;
  lidar_intrinsics_.end_elevation_rad_ = VERTICAL_FOV_MAX;
  lidar_intrinsics_.vertical_fov_ = VERTICAL_FOV_MAX - VERTICAL_FOV_MIN;

  lidar_intrinsics_.start_azimuth_rad_ = HORIZONTAL_FOV_MIN;
  lidar_intrinsics_.end_azimuth_rad_ = HORIZONTAL_FOV_MAX;
  lidar_intrinsics_.horizontal_fov_ = HORIZONTAL_FOV_MAX - HORIZONTAL_FOV_MIN;

  // NOTE(gogojjh): functions to convert pointcloud into image
  pcl::PointCloud<pcl::PointXYZIRRAT>::Ptr cloud_filter_ptr(
      new pcl::PointCloud<pcl::PointXYZIRRAT>());
  computeLidarIntrinsics(input_cloud_ptr_, cloud_filter_ptr, lidar_intrinsics_);

  cv::Mat depth_img(num_elevation_divisions, num_azimuth_divisions, CV_16UC1,
                    cv::Scalar(0));
  Pointcloud2DepthImage(cloud_filter_ptr, lidar_intrinsics, depth_img);
  cv::imwrite("/tmp/depth_image.png", depth_img);

  cv::Mat height_image(num_elevation_divisions, num_azimuth_divisions, CV_16UC1,
                       cv::Scalar(0));
  Pointcloud2HeightImage(cloud_filter_ptr, lidar_intrinsics, height_image);
  cv::imwrite("/tmp/height_image.png", height_img);

  // Publish
  lidar_info_msg_ptr_->header.frame_id = frame_id;
  lidar_info_msg_ptr_->header.stamp = stamp;
  lidar_info_msg_ptr_->height = lidar_intrinsics_.num_elevation_divisions_;
  lidar_info_msg_ptr_->width = lidar_intrinsics_.num_azimuth_divisions_;
  // TODO: Dirty code, using D to store lidar intrinsic
  lidar_info_msg_ptr_->D = {
      static_cast<float>(lidar_intrinsics_.num_azimuth_divisions_),
      static_cast<float>(lidar_intrinsics_.num_elevation_divisions_),
      lidar_intrinsics_.horizontal_fov_,
      lidar_intrinsics_.vertical_fov_,
      lidar_intrinsics_.start_azimuth_rad_,
      lidar_intrinsics_.end_azimuth_rad_,
      lidar_intrinsics_.start_elevation_rad_,
      lidar_intrinsics_.end_elevation_rad_};

  depth_image_msg_ptr_ =
      cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();
  depth_image_msg_ptr_->header.frame_id = frame_id;
  depth_image_msg_ptr_->header.stamp = stamp;
  depth_pub_.publish(depth_image_msg_ptr_, lidar_info_msg_ptr_);

  height_image_msg_ptr_ =
      cv_bridge::CvImage(std_msgs::Header(), "mono16", height_image)
          .toImageMsg();
  height_image_msg_ptr_->header.frame_id = frame_id;
  height_image_msg_ptr_->header.stamp = stamp;
  height_pub_.publish(height_image_msg_ptr_, lidar_info_msg_ptr_);
}
